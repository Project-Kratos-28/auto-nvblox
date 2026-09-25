#!/usr/bin/env python3
"""Measure topic rates without deserializing messages.

`ros2 topic hz` deserializes every message in Python and badly undercounts large images
(1280x720 depth read ~5 Hz while ~11 Hz was published). This subscribes with raw=True,
so only the arrival time is recorded.

    scripts/topic_rates.py [--seconds 15] /topic_a /topic_b ...
"""
import argparse
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('topics', nargs='+')
    parser.add_argument('--seconds', type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('topic_rates')
    deadline = time.time() + 5.0
    types = {}
    while time.time() < deadline and len(types) < len(args.topics):
        for name, tys in node.get_topic_names_and_types():
            if name in args.topics:
                types[name] = tys[0]
        rclpy.spin_once(node, timeout_sec=0.2)

    stamps = {t: [] for t in args.topics}
    # Best effort + volatile matches both reliable and best-effort publishers.
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
    for t, ty in types.items():
        node.create_subscription(get_message(ty), t, lambda _m, t=t: stamps[t].append(time.time()),
                                 qos, raw=True)

    end = time.time() + args.seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    for t in args.topics:
        s = stamps[t]
        if t not in types:
            print(f'{t:48s} not advertised')
        elif len(s) < 2:
            print(f'{t:48s} {len(s)} messages')
        else:
            print(f'{t:48s} {(len(s) - 1) / (s[-1] - s[0]):6.2f} Hz  ({len(s)} msgs)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
