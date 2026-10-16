#!/usr/bin/env python3
"""Print the latest message of some ROS topics once (debug helper for the demo world).

    gz_probe.py /model/tray_B_L08_T10/pose:PoseStamped /givaudan/tray_B_L08_T10/ee_left/state:String
"""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

TYPES = {'PoseStamped': PoseStamped, 'String': String}


def show(msg):
    if isinstance(msg, PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        return f'x {p.x:+.4f} y {p.y:+.4f} z {p.z:+.4f} q ({q.x:+.3f} {q.y:+.3f} {q.z:+.3f} {q.w:+.3f})'
    return repr(msg.data)


def main():
    rclpy.init()
    node = rclpy.create_node('gz_probe')
    got = {}
    for spec in sys.argv[1:]:
        topic, typ = spec.rsplit(':', 1)
        node.create_subscription(TYPES[typ], topic, lambda m, t=topic: got.__setitem__(t, m), 10)
    end = time.time() + 5
    while time.time() < end and len(got) < len(sys.argv) - 1:
        rclpy.spin_once(node, timeout_sec=0.1)
    for spec in sys.argv[1:]:
        topic = spec.rsplit(':', 1)[0]
        print(f'{topic}: {show(got[topic]) if topic in got else "(nothing in 5 s)"}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
