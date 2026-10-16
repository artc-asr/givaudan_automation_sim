#!/usr/bin/env python3
"""Real-time factor from /clock and the robot's Gazebo world pose, for check_spawn.sh.

    measure_spawn.py <model_name> [--seconds 10]

The pose comes from Gazebo's /world/<world>/dynamic_pose/info, bridged to
/gz_poses (tf2_msgs/TFMessage) by check_spawn.sh: ground truth, any robot.
"""
import argparse
import math
import time

import rclpy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('model')
    ap.add_argument('--seconds', type=float, default=10.0)
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('measure_spawn', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    samples = []
    node.create_subscription(Clock, '/clock', lambda m: samples.append(
        (time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
    pose = {}
    node.create_subscription(TFMessage, '/gz_poses', lambda m: pose.update(
        {t.child_frame_id: t.transform for t in m.transforms}), 10)

    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(samples) > 1:
        (w0, s0), (w1, s1) = samples[0], samples[-1]
        print(f'rtf: {(s1 - s0) / (w1 - w0):.2f} ({s1 - s0:.2f} s sim in {w1 - w0:.2f} s wall)')
    else:
        print('rtf: no /clock')
    t = pose.get(a.model)
    if t is None:
        print(f'pose: {a.model} not in /gz_poses (got {sorted(pose)[:5]})')
    else:
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        print(f'pose (gz world): x {t.translation.x:+.3f} y {t.translation.y:+.3f} '
              f'z {t.translation.z:+.3f} yaw {math.degrees(yaw):+.1f} deg')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
