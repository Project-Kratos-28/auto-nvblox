#!/usr/bin/env python3
"""Republish a topic at a lower rate without deserializing it (remote/WLAN viewing).

Onboard consumers keep the full-rate topic; only the throttled copy is offered over the
radio link (see the foxglove whitelist in bringup.launch.py).

Parameters:
    input_topic   topic to read
    output_topic  topic to publish
    max_rate_hz   at most this many messages per second (0 = no limit)
    reliable      publisher reliability (default true; the costmap needs reliable for RViz/Foxglove)
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


class ThrottleRelay(Node):

    def __init__(self):
        super().__init__('throttle_relay')
        self.input_topic = self.declare_parameter('input_topic', '').value
        self.output_topic = self.declare_parameter('output_topic', '').value
        rate = self.declare_parameter('max_rate_hz', 2.0).value
        self.reliable = self.declare_parameter('reliable', True).value
        self.period = 1.0 / rate if rate > 0 else 0.0
        self.last = 0.0
        self.pub = None
        # The type is only known once the publisher is discovered.
        self.timer = self.create_timer(0.5, self.connect)

    def connect(self):
        types = dict(self.get_topic_names_and_types()).get(self.input_topic)
        if not types:
            return
        self.timer.cancel()
        msg_type = get_message(types[0])
        sub_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        pub_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=(ReliabilityPolicy.RELIABLE if self.reliable
                                          else ReliabilityPolicy.BEST_EFFORT),
                             durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(msg_type, self.output_topic, pub_qos)
        self.create_subscription(msg_type, self.input_topic, self.on_msg, sub_qos, raw=True)
        self.get_logger().info(f'{self.input_topic} -> {self.output_topic} ({types[0]}), '
                               f'max {1.0 / self.period if self.period else 0:.1f} Hz')

    def on_msg(self, serialized):
        now = time.monotonic()
        if now - self.last < self.period:
            return
        self.last = now
        self.pub.publish(serialized)   # rclpy publishes serialized bytes as-is


def main():
    rclpy.init()
    node = ThrottleRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
