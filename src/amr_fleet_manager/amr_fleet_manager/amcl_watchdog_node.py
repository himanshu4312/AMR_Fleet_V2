import subprocess

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException
from tf2_ros import TransformListener

AMCL_POSE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

NAVIGATING_STATUSES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}

RVIZ_DEFAULT_INITIALPOSE_COVARIANCE = [0.0] * 36
RVIZ_DEFAULT_INITIALPOSE_COVARIANCE[0] = 0.25
RVIZ_DEFAULT_INITIALPOSE_COVARIANCE[7] = 0.25
RVIZ_DEFAULT_INITIALPOSE_COVARIANCE[35] = 0.06853891945200942

DEFAULT_ROBOT_NAMES = ['robot1', 'robot2', 'robot3', 'robot4']

DEFAULT_STALENESS_THRESHOLD_S = 8.0

DEFAULT_CHECK_PERIOD_S = 2.0

DEFAULT_TF_WARMUP_GRACE_S = 10.0

DEFAULT_RECOVERY_COOLDOWN_S = 15.0

DEFAULT_RESTART_ESCALATION_ATTEMPTS = 2

DEFAULT_RESTART_COOLDOWN_S = 30.0


class RobotWatch:

    def __init__(self, name, tf_node, created_time):
        self.name = name
        self.tf_node = tf_node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, tf_node)
        self.created_time = created_time
        self.last_good_pose = None
        self.is_navigating = False
        self.last_recovery_time = None
        self.stamp_at_last_recovery = None
        self.failed_recovery_count = 0
        self.last_restart_time = None


class AmclWatchdogNode(Node):

    def __init__(self):
        super().__init__('amcl_watchdog_node')

        self.declare_parameter('robot_names', DEFAULT_ROBOT_NAMES)
        self.declare_parameter('staleness_threshold_s', DEFAULT_STALENESS_THRESHOLD_S)
        self.declare_parameter('check_period_s', DEFAULT_CHECK_PERIOD_S)
        self.declare_parameter('recovery_cooldown_s', DEFAULT_RECOVERY_COOLDOWN_S)
        self.declare_parameter('restart_escalation_attempts', DEFAULT_RESTART_ESCALATION_ATTEMPTS)
        self.declare_parameter('restart_cooldown_s', DEFAULT_RESTART_COOLDOWN_S)
        self.declare_parameter('tf_warmup_grace_s', DEFAULT_TF_WARMUP_GRACE_S)

        self.robot_names = list(self.get_parameter('robot_names').value)
        self.staleness_threshold_s = float(self.get_parameter('staleness_threshold_s').value)
        self.check_period_s = float(self.get_parameter('check_period_s').value)
        self.recovery_cooldown_s = float(self.get_parameter('recovery_cooldown_s').value)
        self.restart_escalation_attempts = int(
            self.get_parameter('restart_escalation_attempts').value)
        self.restart_cooldown_s = float(self.get_parameter('restart_cooldown_s').value)
        self.tf_warmup_grace_s = float(self.get_parameter('tf_warmup_grace_s').value)

        self.watches = {}
        self.initialpose_pubs = {}
        self.relocalize_clients = {}
        for name in self.robot_names:
            tf_node = rclpy.create_node(
                f'{name}_tf_listener',
                cli_args=[
                    '--ros-args',
                    '-r', f'/tf:=/{name}/tf',
                    '-r', f'/tf_static:=/{name}/tf_static',
                ],
            )
            self.watches[name] = RobotWatch(name, tf_node, self.get_clock().now())
            self.initialpose_pubs[name] = self.create_publisher(
                PoseWithCovarianceStamped, f'/{name}/initialpose', 10)
            self.relocalize_clients[name] = self.create_client(
                Empty, f'/{name}/reinitialize_global_localization')
            self.create_subscription(
                PoseWithCovarianceStamped, f'/{name}/amcl_pose',
                self._pose_callback(name), AMCL_POSE_QOS)
            self.create_subscription(
                GoalStatusArray, f'/{name}/navigate_to_pose/_action/status',
                self._status_callback(name), 10)

        self.timer = self.create_timer(self.check_period_s, self._check_all)
        self.get_logger().info(
            f'amcl_watchdog watching {self.robot_names}: '
            f'staleness_threshold={self.staleness_threshold_s}s, '
            f'check_period={self.check_period_s}s, '
            f'recovery_cooldown={self.recovery_cooldown_s}s, '
            f'restart_escalation_attempts={self.restart_escalation_attempts}, '
            f'restart_cooldown={self.restart_cooldown_s}s, '
            f'tf_warmup_grace={self.tf_warmup_grace_s}s')

    @property
    def tf_listener_nodes(self):
        return [watch.tf_node for watch in self.watches.values()]

    def _pose_callback(self, name):
        def cb(msg):
            self.watches[name].last_good_pose = msg
        return cb

    def _status_callback(self, name):
        def cb(msg):
            if msg.status_list:
                latest = msg.status_list[-1]
                self.watches[name].is_navigating = latest.status in NAVIGATING_STATUSES
        return cb

    def _check_all(self):
        for name in self.robot_names:
            self._check_one(name)

    def _check_one(self, name):
        watch = self.watches[name]
        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9

        try:
            transform = watch.tf_buffer.lookup_transform('map', f'{name}/odom', Time())
            stamp_s = Time.from_msg(transform.header.stamp).nanoseconds / 1e9
        except (LookupException, ConnectivityException, ExtrapolationException):
            since_created_s = now_s - watch.created_time.nanoseconds / 1e9
            if since_created_s < self.tf_warmup_grace_s:
                return
            if watch.last_good_pose is None:
                return
            stamp_s = Time.from_msg(watch.last_good_pose.header.stamp).nanoseconds / 1e9

        age_s = now_s - stamp_s

        if age_s < self.staleness_threshold_s:
            watch.failed_recovery_count = 0
            return

        if watch.last_restart_time is not None:
            since_restart_s = now_s - watch.last_restart_time.nanoseconds / 1e9
            if since_restart_s < self.restart_cooldown_s:
                return

        if watch.last_recovery_time is not None:
            since_recovery_s = (now - watch.last_recovery_time).nanoseconds / 1e9
            if since_recovery_s < self.recovery_cooldown_s:
                return

            if watch.stamp_at_last_recovery is not None \
                    and stamp_s <= watch.stamp_at_last_recovery:
                watch.failed_recovery_count += 1
            else:
                watch.failed_recovery_count = 0

        if watch.failed_recovery_count >= self.restart_escalation_attempts:
            self._restart_amcl(name, watch, now)
            return

        if watch.is_navigating:
            self._relocalize(name, watch, now, age_s, stamp_s)
        else:
            self._reseed_idle(name, watch, now, age_s, stamp_s)

    def _reseed_idle(self, name, watch, now, age_s, stamp_s):
        if watch.last_good_pose is None:
            return

        self.get_logger().warn(
            f'{name}: pose is {age_s:.1f}s old (threshold '
            f'{self.staleness_threshold_s}s) - AMCL may have stalled while '
            f'idle. Re-publishing its last known pose to '
            f'/{name}/initialpose to force a reseed.')

        watch.stamp_at_last_recovery = stamp_s
        watch.last_recovery_time = now

        reseed_msg = PoseWithCovarianceStamped()
        reseed_msg.header.stamp = now.to_msg()
        reseed_msg.header.frame_id = 'map'
        reseed_msg.pose.pose = watch.last_good_pose.pose.pose
        reseed_msg.pose.covariance = list(RVIZ_DEFAULT_INITIALPOSE_COVARIANCE)
        self.initialpose_pubs[name].publish(reseed_msg)

    def _relocalize(self, name, watch, now, age_s, stamp_s):
        self.get_logger().warn(
            f'{name}: pose is {age_s:.1f}s old (threshold '
            f'{self.staleness_threshold_s}s) - AMCL may have stalled while '
            f'a goal is in flight. Calling '
            f'/{name}/reinitialize_global_localization to force it to '
            f're-converge from live scans (its cached pose can no longer '
            f'be trusted while the robot may still be moving).')

        watch.stamp_at_last_recovery = stamp_s
        watch.last_recovery_time = now

        client = self.relocalize_clients[name]
        if not client.service_is_ready():
            self.get_logger().warn(
                f'{name}: reinitialize_global_localization service is not '
                f'available yet - will retry next cooldown.')
            return
        client.call_async(Empty.Request())

    def _restart_amcl(self, name, watch, now):
        self.get_logger().error(
            f'{name}: {watch.failed_recovery_count} recovery attempt(s) '
            f'produced no response at all - AMCL appears completely wedged, '
            f'not just stalled. Killing its process; amcl_node.respawn=True '
            f'in multi_robot_bringup.launch.py will bring up a fresh '
            f'instance with the same launch parameters.')

        pattern = f'nav2_amcl/amcl .*__ns:=/{name} '
        subprocess.run(['pkill', '-9', '-f', pattern], check=False)

        watch.failed_recovery_count = 0
        watch.last_recovery_time = None
        watch.stamp_at_last_recovery = None
        watch.last_restart_time = now


def main(args=None):
    rclpy.init(args=args)
    node = AmclWatchdogNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    for tf_node in node.tf_listener_nodes:
        executor.add_node(tf_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for tf_node in node.tf_listener_nodes:
            tf_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
