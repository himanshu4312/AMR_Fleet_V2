import math
import subprocess

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PolygonStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from nav2_msgs.msg import SpeedLimit

from amr_fleet_msgs.msg import FleetRobotState

DEFAULT_ROBOT_NAMES = ['robot1', 'robot2', 'robot3', 'robot4']


DEFAULT_SAFETY_DISTANCE_M = 2.0

DEFAULT_TIME_WINDOW_S = 4.0

TIME_TIE_EPSILON_S = 0.1
DISTANCE_TIE_EPSILON_M = 0.02

DEFAULT_NOMINAL_SPEED_MPS = 0.26

DEFAULT_PATH_CHECK_STRIDE = 1

TIMER_PERIOD_S = 0.5

PAUSE_SPEED_LIMIT_PERCENT = 1.0
RESUME_SPEED_LIMIT_PERCENT = 0.0

FALSE_SUCCESS_TOLERANCE_M = 0.75

MAX_AUTO_RESEND_ATTEMPTS = 3

MAX_CONTROLLER_RESTART_ATTEMPTS = 2

RESEND_DELAY_S = 3.0

RESEND_RETRY_PERIOD_S = 5.0
RESEND_RETRY_LIMIT = 24

NAVIGATING_STATUSES = (
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
)

RESTART_COOLDOWN_S = 20.0

COSTMAP_STALE_THRESHOLD_S = 12.0

FROZEN_RESTART_COOLDOWN_S = 45.0
MAX_FROZEN_RESTARTS = 3

CLOCK_MISMATCH_AGE_S = 1.0e6

HEALTHY_RESET_S = 90.0


def _dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def _clamp01(x):
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def _closest_pt_segment_segment(p1, p2, q1, q2):
    d1x, d1y = p2[0] - p1[0], p2[1] - p1[1]
    d2x, d2y = q2[0] - q1[0], q2[1] - q1[1]
    rx, ry = p1[0] - q1[0], p1[1] - q1[1]

    a = d1x * d1x + d1y * d1y
    e = d2x * d2x + d2y * d2y
    f = d2x * rx + d2y * ry

    eps = 1e-9
    if a <= eps and e <= eps:
        s = t = 0.0
    elif a <= eps:
        s = 0.0
        t = _clamp01(f / e)
    else:
        c = d1x * rx + d1y * ry
        if e <= eps:
            t = 0.0
            s = _clamp01(-c / a)
        else:
            b = d1x * d2x + d1y * d2y
            denom = a * e - b * b
            s = _clamp01((b * f - c * e) / denom) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = _clamp01(-c / a)
            elif t > 1.0:
                t = 1.0
                s = _clamp01((b - c) / a)

    c1 = (p1[0] + d1x * s, p1[1] + d1y * s)
    c2 = (q1[0] + d2x * t, q1[1] + d2y * t)
    return math.hypot(c1[0] - c2[0], c1[1] - c2[1]), s, t


class RobotTrack:

    def __init__(self, name):
        self.name = name
        self.pose = None
        self.points = []
        self.cumulative = []
        self.last_goal_pose = None
        self.last_reacted_goal_id = None
        self.false_success_streak = 0
        self.controller_restart_count = 0
        self.is_navigating = None
        self.last_status = None
        self.footprint_stamp_s = None
        self.footprint_ignore_before_s = 0.0
        self.last_frozen_restart_s = None
        self.frozen_restart_count = 0

    @property
    def remaining_path_length(self):
        return self.cumulative[-1] if self.cumulative else 0.0

    def set_path(self, path_msg: Path):
        points = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
        cumulative = []
        total = 0.0
        for i, pt in enumerate(points):
            if i > 0:
                total += _dist(points[i - 1], pt)
            cumulative.append(total)
        self.points = points
        self.cumulative = cumulative
        if path_msg.poses:
            self.last_goal_pose = path_msg.poses[-1]


class FleetManagerNode(Node):

    def __init__(self):
        super().__init__('fleet_manager_node')

        self.declare_parameter('robot_names', DEFAULT_ROBOT_NAMES)
        self.declare_parameter('safety_distance_m', DEFAULT_SAFETY_DISTANCE_M)
        self.declare_parameter('time_window_s', DEFAULT_TIME_WINDOW_S)
        self.declare_parameter('nominal_speed_mps', DEFAULT_NOMINAL_SPEED_MPS)
        self.declare_parameter('path_check_stride', DEFAULT_PATH_CHECK_STRIDE)

        self.robot_names = list(self.get_parameter('robot_names').value)
        self.safety_distance_m = float(self.get_parameter('safety_distance_m').value)
        self.time_window_s = float(self.get_parameter('time_window_s').value)
        self.nominal_speed_mps = float(self.get_parameter('nominal_speed_mps').value)
        self.path_check_stride = max(1, int(self.get_parameter('path_check_stride').value))

        self.tracks = {name: RobotTrack(name) for name in self.robot_names}
        self._warned_clock_mismatch = False

        self.active_conflicts = {}

        self.speed_limit_pubs = {}
        self.nav_action_clients = {}
        for name in self.robot_names:
            self.speed_limit_pubs[name] = self.create_publisher(
                SpeedLimit, f'/{name}/speed_limit', 10)
            self.create_subscription(
                Path, f'/{name}/plan', self._plan_callback(name), 10)
            self.create_subscription(
                PoseWithCovarianceStamped, f'/{name}/amcl_pose', self._pose_callback(name), 10)
            self.create_subscription(
                PolygonStamped, f'/{name}/local_costmap/published_footprint',
                self._footprint_callback(name), 10)
            self.create_subscription(
                GoalStatusArray, f'/{name}/navigate_to_pose/_action/status',
                self._goal_status_callback(name), 10)
            self.nav_action_clients[name] = ActionClient(
                self, NavigateToPose, f'/{name}/navigate_to_pose')

        self.state_pub = self.create_publisher(FleetRobotState, '/fleet_manager/robot_states', 10)

        for name in self.robot_names:
            self._publish_speed_limit(name, resume=True)

        self.timer = self.create_timer(TIMER_PERIOD_S, self.on_timer)
        self.get_logger().info(
            f'fleet_manager watching {self.robot_names}: '
            f'safety_distance={self.safety_distance_m}m, '
            f'time_window={self.time_window_s}s, '
            f'nominal_speed={self.nominal_speed_mps}m/s')


    def _plan_callback(self, name):
        def cb(msg: Path):
            self.tracks[name].set_path(msg)
        return cb

    def _pose_callback(self, name):
        def cb(msg: PoseWithCovarianceStamped):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self.tracks[name].pose = pose
        return cb

    def _footprint_callback(self, name):
        def cb(msg: PolygonStamped):
            stamp = msg.header.stamp
            stamp_s = stamp.sec + stamp.nanosec * 1e-9
            track = self.tracks[name]
            if stamp_s < track.footprint_ignore_before_s:
                return
            track.footprint_stamp_s = stamp_s
        return cb

    def _goal_status_callback(self, name):
        def cb(msg: GoalStatusArray):
            if not msg.status_list:
                return
            track = self.tracks[name]
            navigating_now = any(
                s.status in NAVIGATING_STATUSES for s in msg.status_list)
            if navigating_now and track.is_navigating is False:
                track.points = []
                track.cumulative = []
            track.is_navigating = navigating_now
            track.last_status = msg.status_list[-1].status

            latest = msg.status_list[-1]
            if latest.status != GoalStatus.STATUS_SUCCEEDED:
                return
            goal_id = bytes(latest.goal_info.goal_id.uuid)
            if goal_id == track.last_reacted_goal_id:
                return
            track.last_reacted_goal_id = goal_id
            self._verify_goal_reached(name)
        return cb


    def _verify_goal_reached(self, name):
        track = self.tracks[name]
        if track.pose is None or track.last_goal_pose is None:
            return

        actual = track.pose.pose.position
        goal = track.last_goal_pose.pose.position
        error_m = _dist((actual.x, actual.y), (goal.x, goal.y))

        if error_m <= FALSE_SUCCESS_TOLERANCE_M:
            track.false_success_streak = 0
            track.controller_restart_count = 0
            track.frozen_restart_count = 0
            return

        if track.false_success_streak >= MAX_AUTO_RESEND_ATTEMPTS:
            track.false_success_streak = 0
            if track.controller_restart_count >= MAX_CONTROLLER_RESTART_ATTEMPTS:
                self.get_logger().error(
                    f'{name}: still false-succeeding {error_m:.2f}m from its '
                    f'goal after {track.controller_restart_count} '
                    f'controller_server restart(s) - giving up. This robot '
                    f'needs manual investigation.')
                return
            track.controller_restart_count += 1
            self.get_logger().error(
                f'{name}: {MAX_AUTO_RESEND_ATTEMPTS} resends to the same '
                f'controller_server process all failed identically - it is '
                f'likely permanently corrupted (not an AMCL/TF issue, which '
                f'a resend would have fixed). Restarting its process '
                f'(attempt {track.controller_restart_count}/'
                f'{MAX_CONTROLLER_RESTART_ATTEMPTS}).')
            self._restart_controller_server(name)
            return

        track.false_success_streak += 1
        self.get_logger().warn(
            f'{name}: reported SUCCEEDED but is {error_m:.2f}m from its goal '
            f'(tolerance {FALSE_SUCCESS_TOLERANCE_M}m) - likely the stale-TF '
            f'false-success defect, not a real arrival. Re-sending the same '
            f'goal in {RESEND_DELAY_S}s '
            f'(attempt {track.false_success_streak}/{MAX_AUTO_RESEND_ATTEMPTS}).')
        self._schedule_resend(name, RESEND_DELAY_S)

    def _check_controller_health(self):
        now_s = self.get_clock().now().nanoseconds / 1e9
        for name in self.robot_names:
            track = self.tracks[name]
            if track.footprint_stamp_s is None:
                continue
            if track.last_frozen_restart_s is not None \
                    and now_s - track.last_frozen_restart_s < FROZEN_RESTART_COOLDOWN_S:
                continue
            age_s = now_s - track.footprint_stamp_s
            if age_s > CLOCK_MISMATCH_AGE_S:
                if not self._warned_clock_mismatch:
                    self._warned_clock_mismatch = True
                    self.get_logger().error(
                        'costmap footprint stamps are on a different clock '
                        'than this node (age > 1e6 s). Start this node with '
                        '`--ros-args -p use_sim_time:=true`. Controller '
                        'health monitoring is DISABLED until then.')
                return
            if age_s < COSTMAP_STALE_THRESHOLD_S:
                if track.frozen_restart_count \
                        and track.last_frozen_restart_s is not None \
                        and now_s - track.last_frozen_restart_s > HEALTHY_RESET_S:
                    track.frozen_restart_count = 0
                continue

            if track.frozen_restart_count >= MAX_FROZEN_RESTARTS:
                track.last_frozen_restart_s = now_s
                self.get_logger().error(
                    f'{name}: controller_server looks frozen again (local '
                    f'costmap footprint {age_s:.0f}s old) after '
                    f'{track.frozen_restart_count} restart(s) - giving up. '
                    f'This robot needs manual investigation.')
                continue

            resume = track.is_navigating or track.last_status == GoalStatus.STATUS_ABORTED
            resume = bool(resume) and track.last_goal_pose is not None
            track.frozen_restart_count += 1
            track.last_frozen_restart_s = now_s
            self.get_logger().error(
                f'{name}: controller_server appears frozen - its local '
                f'costmap footprint is {age_s:.0f}s old (threshold '
                f'{COSTMAP_STALE_THRESHOLD_S:.0f}s) while AMCL/TF are '
                f'healthy. Restarting it '
                f'(attempt {track.frozen_restart_count}/{MAX_FROZEN_RESTARTS})'
                f'{" and re-sending the goal" if resume else ""}.')
            self._restart_controller_server(name, resend=resume)

    def _restart_controller_server(self, name, resend=True):
        pattern = f'nav2_controller/controller_server .*__ns:=/{name} '
        subprocess.run(['pkill', '-9', '-f', pattern], check=False)
        self.tracks[name].last_frozen_restart_s = self.get_clock().now().nanoseconds / 1e9
        self.tracks[name].footprint_stamp_s = None
        self.tracks[name].footprint_ignore_before_s = \
            self.get_clock().now().nanoseconds / 1e9
        if resend:
            self._schedule_resend(name, RESTART_COOLDOWN_S)

    def _schedule_resend(self, name, delay_s, attempts_left=RESEND_RETRY_LIMIT):
        box = {}

        def fire():
            box['timer'].cancel()
            self._resend_goal(name, attempts_left)

        box['timer'] = self.create_timer(delay_s, fire)

    def _resend_goal(self, name, attempts_left=0):
        track = self.tracks[name]
        client = self.nav_action_clients[name]
        if track.is_navigating:
            return
        if not client.server_is_ready():
            if attempts_left > 0:
                self._schedule_resend(name, RESEND_RETRY_PERIOD_S, attempts_left - 1)
            else:
                self.get_logger().warn(
                    f'{name}: navigate_to_pose action server still not '
                    f'available after retrying - giving up on the auto-resend.')
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = track.last_goal_pose
        client.send_goal_async(goal_msg)


    def on_timer(self):
        self._check_controller_health()
        self._update_conflicts()
        self._publish_states()

    def _update_conflicts(self):
        names = self.robot_names
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = names[i], names[j]
                pair_key = tuple(sorted((a, b)))
                crossing = self._find_conflict_crossing(a, b)

                if pair_key in self.active_conflicts:
                    if crossing is None:
                        paused_robot = self.active_conflicts.pop(pair_key)
                        self.get_logger().info(
                            f'Conflict cleared between {a} and {b}; resuming {paused_robot}')
                elif crossing is not None:
                    dist_a, dist_b, time_a, time_b = crossing
                    winner, loser, reason = self._decide_priority(
                        a, dist_a, time_a, b, dist_b, time_b)
                    self.active_conflicts[pair_key] = loser
                    self.get_logger().info(
                        f'Conflict detected between {a} and {b}: pausing {loser} while '
                        f'{winner} crosses first ({reason})')

        paused_now = set(self.active_conflicts.values())
        for name in names:
            self._publish_speed_limit(name, resume=(name not in paused_now))

    def _decide_priority(self, name_a, dist_a, time_a, name_b, dist_b, time_b):
        if abs(time_a - time_b) >= TIME_TIE_EPSILON_S:
            return (name_a, name_b, 'reaches crossing first') if time_a < time_b \
                else (name_b, name_a, 'reaches crossing first')

        safe_time_a = time_a - self.safety_distance_m / self.nominal_speed_mps
        safe_time_b = time_b - self.safety_distance_m / self.nominal_speed_mps
        if abs(safe_time_a - safe_time_b) >= TIME_TIE_EPSILON_S:
            reason = 'reaches safe-distance boundary first'
            return (name_a, name_b, reason) if safe_time_a < safe_time_b \
                else (name_b, name_a, reason)

        remaining_a = self.tracks[name_a].remaining_path_length
        remaining_b = self.tracks[name_b].remaining_path_length
        if abs(remaining_a - remaining_b) >= DISTANCE_TIE_EPSILON_M:
            return (name_a, name_b, 'closer to its goal') if remaining_a < remaining_b \
                else (name_b, name_a, 'closer to its goal')

        if self.robot_names.index(name_a) < self.robot_names.index(name_b):
            return name_a, name_b, 'lower robot number'
        return name_b, name_a, 'lower robot number'

    def _find_conflict_crossing(self, name_a, name_b):
        track_a = self.tracks[name_a]
        track_b = self.tracks[name_b]
        if track_a.is_navigating is False or track_b.is_navigating is False:
            return None
        if len(track_a.points) < 2 or len(track_b.points) < 2:
            return None

        stride = self.path_check_stride
        for i in range(0, len(track_a.points) - 1, stride):
            a1, a2 = track_a.points[i], track_a.points[i + 1]
            seg_len_a = _dist(a1, a2)
            for j in range(0, len(track_b.points) - 1, stride):
                b1, b2 = track_b.points[j], track_b.points[j + 1]

                dist, s, t = _closest_pt_segment_segment(a1, a2, b1, b2)
                if dist >= self.safety_distance_m:
                    continue

                seg_len_b = _dist(b1, b2)
                dist_a = track_a.cumulative[i] + s * seg_len_a
                dist_b = track_b.cumulative[j] + t * seg_len_b
                time_a = dist_a / self.nominal_speed_mps
                time_b = dist_b / self.nominal_speed_mps

                if abs(time_a - time_b) < self.time_window_s:
                    return dist_a, dist_b, time_a, time_b
        return None

    def _publish_speed_limit(self, name, resume: bool):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.percentage = True
        msg.speed_limit = RESUME_SPEED_LIMIT_PERCENT if resume else PAUSE_SPEED_LIMIT_PERCENT
        self.speed_limit_pubs[name].publish(msg)

    def _publish_states(self):
        paused_now = set(self.active_conflicts.values())
        ranked = sorted(self.robot_names, key=lambda n: self.tracks[n].remaining_path_length)
        rank_of = {name: idx + 1 for idx, name in enumerate(ranked)}

        for name in self.robot_names:
            track = self.tracks[name]
            if track.pose is None:
                continue
            msg = FleetRobotState()
            msg.robot_name = name
            msg.pose = track.pose
            msg.remaining_path_length = track.remaining_path_length
            msg.is_paused = name in paused_now
            msg.priority_rank = rank_of[name]
            self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
