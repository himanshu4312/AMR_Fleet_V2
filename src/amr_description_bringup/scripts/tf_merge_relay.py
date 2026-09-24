#!/usr/bin/env python3

from tf2_msgs.msg import TFMessage
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def make_relay(node: Node, robot_name: str, src_topic: str, dst_topic: str, transient_local: bool):
    durability = DurabilityPolicy.TRANSIENT_LOCAL if transient_local else DurabilityPolicy.VOLATILE
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=durability)
    pub = node.create_publisher(TFMessage, dst_topic, qos)

    def cb(msg: TFMessage):
        pub.publish(msg)

    node.create_subscription(TFMessage, src_topic, cb, qos)


def main():
    rclpy.init()
    node = Node('tf_merge_relay')
    for robot_name in ('robot1', 'robot2', 'robot3', 'robot4'):
        make_relay(node, robot_name, f'/{robot_name}/tf', '/tf', transient_local=False)
        make_relay(node, robot_name, f'/{robot_name}/tf_static', '/tf_static', transient_local=True)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
