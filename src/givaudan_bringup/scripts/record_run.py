#!/usr/bin/env python3
"""Record a demo run for the web replay (docs/), adapted from tccc_hmlv_automation_sim.

    ros2 launch givaudan_bringup demo.launch.py plan:=... record:=/tmp/run_task1

Samples at `rate` Hz of sim time: the robot's joints (/joint_states), its base pose
(TF world -> base_frame) and the poses of the moving objects (/model/<name>/pose from
Gazebo's PosePublisher). Also keeps, with their sim times, the demo's steps
(/givaudan_demo/phase) and its log lines (/rosout, node givaudan_demo). Stops `tail` s
after "done" or "failed" (or on Ctrl-C) and writes to `out_dir`:

    recording.json   the samples and events
    robot.urdf       /robot_description as the sim used it
    world.sdf        a copy of world_file (the static scene)
    plan.json        a copy of the plan (step labels for the page)

tools/web_replay/build.py turns that into the page's data.
"""
import json
import os
import shutil
import signal

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def _pose(p, q):
    return [round(v, 4) for v in (p.x, p.y, p.z)] + [round(v, 5) for v in (q.x, q.y, q.z, q.w)]


class RunRecorder(Node):
    def __init__(self):
        super().__init__('record_run')
        self.out_dir = self.declare_parameter('out_dir', '/tmp/givaudan_run').value
        self.world_file = self.declare_parameter('world_file', '').value
        self.plan_file = self.declare_parameter('plan_file', '').value
        self.run_id = self.declare_parameter('run_id', '').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        objects = [o for o in self.declare_parameter('objects', ['']).value if o]
        rate = self.declare_parameter('rate', 20.0).value
        self.tail = self.declare_parameter('tail', 3.0).value
        self.max_duration = self.declare_parameter('max_duration', 900.0).value

        self.joints, self.base, self.objects, self.urdf = {}, None, {}, None
        self.t0 = self.end_at = None
        self.written = False
        self.rec = {'rate': rate, 'joint_names': [], 't': [], 'base': [], 'joints': [],
                    'objects': {}, 'phases': [], 'logs': [], 'base_frame': self.base_frame}
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(JointState, '/joint_states', lambda m: self.joints.update(zip(m.name, m.position)), 50)
        self.create_subscription(String, '/robot_description', lambda m: setattr(self, 'urdf', m.data), LATCHED)
        self.create_subscription(String, '/givaudan_demo/phase', self._on_phase, LATCHED)
        self.create_subscription(Log, '/rosout', self._on_log, 100)
        for name in objects:
            self.create_subscription(PoseStamped, f'/model/{name}/pose',
                                     lambda m, n=name: self.objects.__setitem__(n, _pose(m.pose.position, m.pose.orientation)), 10)
        self.create_timer(1.0 / rate, self._sample)
        self.get_logger().info(f'recording at {rate:.0f} Hz into {self.out_dir}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _rel(self, t):
        return round(t - self.t0, 3) if self.t0 is not None else 0.0

    def _on_phase(self, msg):
        self.rec['phases'].append({'t': self._rel(self._now()), 'phase': msg.data})
        if msg.data in ('done', 'failed') and self.end_at is None:
            self.end_at = self._now() + self.tail

    def _on_log(self, msg):
        if msg.name != 'givaudan_demo':
            return
        # at receipt, in sim time: /rosout stamps are wall-clock time
        self.rec['logs'].append({'t': self._rel(self._now()), 'level': int(msg.level), 'msg': msg.msg})

    def _sample(self):
        now = self._now()
        try:
            t = self.tf.lookup_transform('world', self.base_frame, Time()).transform
            self.base = _pose(t.translation, t.rotation)
        except Exception:  # noqa: BLE001  (TF not up yet)
            pass
        if now <= 0.0 or self.base is None or not self.joints:
            return
        if self.t0 is None:
            self.t0 = now
            self.rec['joint_names'] = sorted(self.joints)
            self.get_logger().info(f'first sample: {len(self.joints)} joints, {len(self.objects)} objects')
        r = self.rec
        r['t'].append(self._rel(now))
        r['base'].append(self.base)
        r['joints'].append([round(self.joints.get(n, 0.0), 4) for n in r['joint_names']])
        n = len(r['t'])
        for name, pose in self.objects.items():
            r['objects'].setdefault(name, [pose] * (n - 1)).append(pose)   # back-fill a late first pose
        if (self.end_at is not None and now >= self.end_at) or now - self.t0 > self.max_duration:
            self.write()
            raise SystemExit

    def write(self):
        if self.written or self.t0 is None:
            return
        self.written = True
        os.makedirs(self.out_dir, exist_ok=True)
        self.rec['run_id'] = self.run_id
        if self.urdf:
            with open(os.path.join(self.out_dir, 'robot.urdf'), 'w') as f:
                f.write(self.urdf)
        for src, dst in ((self.world_file, 'world.sdf'), (self.plan_file, 'plan.json')):
            if src:
                shutil.copy(src, os.path.join(self.out_dir, dst))
        # last and atomic: recording.json appearing means the run is complete
        tmp = os.path.join(self.out_dir, 'recording.json.tmp')
        with open(tmp, 'w') as f:
            json.dump(self.rec, f, separators=(',', ':'))
        os.replace(tmp, os.path.join(self.out_dir, 'recording.json'))
        self.get_logger().info(f"wrote {len(self.rec['t'])} samples ({self.rec['t'][-1]:.0f} s), "
                               f"{len(self.rec['phases'])} phases, {len(self.rec['logs'])} log lines to {self.out_dir}")


def main():
    rclpy.init()
    node = RunRecorder()

    def on_sigint(*_):              # ros2 launch stops nodes with SIGINT: keep what there is
        node.write()
        raise SystemExit
    signal.signal(signal.SIGINT, on_sigint)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        node.write()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
