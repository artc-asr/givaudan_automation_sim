#!/usr/bin/env python3
"""Run a planned shelf demo (reachability/plan_demo.py) in Gazebo.

    ros2 launch givaudan_bringup demo.launch.py plan:=reachability/plans/task1_L08.json

Per step: publishes the step on /givaudan_demo/phase, sends its joint trajectory to the
torso, left-arm and right-arm JointTrajectoryControllers at once (same time stamps), waits,
then runs the step's events ("attach tray ee_left": the sim-only weld from the world's
DetachableJoint). Logs what Gazebo measured (tray / vial poses) against the plan.

At the start: the world's welds attach as soon as the robot exists (Harmonic's
DetachableJoint), so all are detached and the tray and vial put back in their slots.
"""
import json
import math
import subprocess
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

CHECK_TOL = 0.01           # m: a hand farther than this from its planned grasp pose has not grasped
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
WORLD = 'shelf_aisle'


def gz_pub(topic):
    subprocess.run(['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Empty', '-p', ' '], check=True, timeout=10)


def gz_set_pose(name, xyz, yaw):
    q = f'x: 0, y: 0, z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}'
    req = f'name: "{name}", position: {{x: {xyz[0]}, y: {xyz[1]}, z: {xyz[2]}}}, orientation: {{{q}}}'
    out = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/set_pose', '--reqtype', 'gz.msgs.Pose',
                          '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                         capture_output=True, text=True, timeout=10)
    return 'true' in out.stdout


class DemoRunner(Node):
    def __init__(self):
        super().__init__('givaudan_demo')
        self.plan = json.load(open(self.declare_parameter('plan', '').value))
        self.speed = float(self.declare_parameter('speed', 1.0).value)    # >1: faster than planned
        # hand frame -> the Gazebo link the welds use (fixed-joint links are merged in Gazebo)
        self.gz_link = {f: self.declare_parameter(f'gz_link_{f}', f).value for f in ('ee_left', 'ee_right')}
        self.phase_pub = self.create_publisher(String, '/givaudan_demo/phase', LATCHED)
        self.poses = {}
        for model in (self.plan['tray'], self.plan['vial']):
            self.create_subscription(PoseStamped, f'/model/{model}/pose',
                                     lambda m, k=model: self.poses.__setitem__(k, m.pose), 10)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.joints = {}
        self.create_subscription(JointState, '/joint_states', lambda m: self.joints.update(zip(m.name, m.position)), 50)
        self.traj_clients = {g: ActionClient(self, FollowJointTrajectory, f'/{g}_controller/follow_joint_trajectory')
                             for g in {**self.plan['groups'], **self.plan.get('finger_groups', {})}}

    # ---------------------------------------------------------------- helpers
    def phase(self, key):
        self.phase_pub.publish(String(data=key))

    def wait(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self, model):
        self.wait(0.3)
        p = self.poses.get(model)
        return None if p is None else (p.position.x, p.position.y, p.position.z)

    def run_traj(self, times, qs, names=None, groups=None):
        names = names or self.plan['joint_names']
        futures = []
        for group, joints in (groups or self.plan['groups']).items():
            if not set(joints) <= set(names):
                continue                      # a group this command does not move
            idx = [names.index(j) for j in joints]
            traj = JointTrajectory(joint_names=joints)
            for t, q in zip(times, qs):
                t = t / self.speed
                traj.points.append(JointTrajectoryPoint(
                    positions=[q[i] for i in idx],
                    time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9))))
            goal = FollowJointTrajectory.Goal(trajectory=traj)
            futures.append(self.traj_clients[group].send_goal_async(goal))
        handles = []
        for f in futures:
            rclpy.spin_until_future_complete(self, f)
            if not f.result().accepted:
                raise RuntimeError('trajectory goal rejected')
            handles.append(f.result().get_result_async())
        for h in handles:
            rclpy.spin_until_future_complete(self, h)
            if h.result().result.error_code != 0:
                raise RuntimeError(f'trajectory failed: {h.result().result.error_string or h.result().result.error_code}')

    def settle(self, q_final, tol=0.01, timeout=4.0):
        """Wait until the measured joints reach the step's last point. The trajectory controller
        reports success when the trajectory's time is up; under load the arm is still behind
        then (task 1 pull: 18.8 of 25 cm), so the next step would start from the wrong place."""
        names, start = self.plan['joint_names'], time.time()
        err = float('inf')
        while time.time() - start < timeout:
            self.wait(0.1)
            if all(n in self.joints for n in names):
                err = max(abs(self.joints[n] - q) for n, q in zip(names, q_final))
                if err < tol:
                    break
        return time.time() - start, err

    def fingers(self, positions, seconds=1.0):
        """Set the grippers (positions: {joint: m}) in `seconds`."""
        names = list(positions)
        self.run_traj([0.0, seconds], [[self.joints.get(n, 0.0) for n in names], [positions[n] for n in names]],
                      names, self.plan['finger_groups'])

    def check(self, step, joint_err):
        """Is the hand where the plan put it? Before welding: a weld would otherwise hide a hand
        that never reached the handle / vial. Returns an error message or ''."""
        c = step.get('check')
        if not c:
            return ''
        try:
            t = self.tf.lookup_transform('world', c['frame'], Time()).transform.translation
        except Exception as e:  # noqa: BLE001
            return f'no TF for {c["frame"]}: {e}'
        d = [t.x - c['pos'][0], t.y - c['pos'][1], t.z - c['pos'][2]]
        dist = math.sqrt(sum(v * v for v in d))
        hand = 'left hand' if 'left' in c['frame'] else 'right hand'
        self.get_logger().info(f'{hand} is {dist * 1000:.0f} mm from its planned pose '
                               f'(x {d[0] * 1000:+.0f}, y {d[1] * 1000:+.0f}, z {d[2] * 1000:+.0f} mm)')
        if dist > CHECK_TOL:
            return (f'{hand} stopped {dist * 1000:.0f} mm short of its target, its arm blocked '
                    f'({joint_err:.2f} rad off the plan after waiting): no grasp')
        return ''

    def event(self, ev):
        verb, what, link = ev.split()
        model = self.plan['tray'] if what == 'tray' else self.plan['vial']
        gz_pub(f'/givaudan/{model}/{self.gz_link[link]}/{verb}')
        if what == 'tray' and verb == 'attach' and self.plan.get('fingers'):
            side = 'l' if link == 'ee_left' else 'r'      # squeeze the fingers 1 mm onto the bar / tray
            f = {k: v - 0.001 * (1 if v > 0 else -1) for k, v in self.plan['fingers'].items() if f'_{side}' in k}
            self.fingers(f, 0.5)
        self.get_logger().info(f'{"suction on" if what == "vial" else "grip closed"}: {model} welded to {link}')

    # ---------------------------------------------------------------- run
    def run(self):
        p = self.plan
        for c in self.traj_clients.values():
            c.wait_for_server()
        self.get_logger().info(f"task {p['task']}: level {p['level']} (board top {p['board_top']:.2f} m), "
                               f"base at ({p['base']['x']:+.2f}, {p['base']['y']:+.2f}) m, "
                               f"{math.degrees(p['base']['yaw']):+.0f} deg")
        # the welds attached when the robot appeared: release them, put tray and vial back
        for model, links in ((p['tray'], ('ee_left', 'ee_right')), (p['vial'], ('ee_right',))):
            for link in links:
                gz_pub(f'/givaudan/{model}/{self.gz_link[link]}/detach')
        self.wait(1.0)
        th = p['tray_home']
        ok = gz_set_pose(p['tray'], (th['x'], th['y'], th['z']), th['yaw'])
        ok &= gz_set_pose(p['vial'], p['vial_home'], 0.0)
        self.wait(2.0)
        if p.get('fingers'):            # the finger opening the plan's collision model assumed
            self.fingers(p['fingers'])
            self.get_logger().info('grippers set: ' + ', '.join(f'{k} {v * 1000:+.0f} mm' for k, v in p['fingers'].items()))
        tray0, vial0 = self.pose(p['tray']), self.pose(p['vial'])
        self.get_logger().info(f'tray and vial in their slots (reset {"ok" if ok else "FAILED"}); '
                               f'tray at x {tray0[0]:.3f} m' if tray0 else 'no tray pose from Gazebo')
        for step in p['steps']:
            self.phase(step['phase'])
            self.get_logger().info(f"{step['label']} ({step['times'][-1] / self.speed:.1f} s)")
            try:
                self.run_traj(step['times'], step['q'])
            except RuntimeError as e:
                self.get_logger().error(f'stopped: {e}')
                self.phase('failed')
                return
            waited, err = self.settle(step['q'][-1])
            if waited > 0.3:
                self.get_logger().info(f'arm caught up with the plan in {waited:.1f} s (joint error {err:.3f} rad)')
            problem = self.check(step, err)
            if problem:
                self.get_logger().error(f'stopped: {problem}')
                self.phase('failed')
                return
            for ev in step['events']:
                self.event(ev)
            self.wait(0.5)
            self.report(step['phase'], tray0, vial0)
        self.phase('done')
        self.get_logger().info('done')

    def report(self, key, tray0, vial0):
        p = self.plan
        tray, vial = self.pose(p['tray']), self.pose(p['vial'])
        if key in ('pull', 'pull_out') and tray and tray0:
            moved = math.hypot(tray[0] - tray0[0], tray[1] - tray0[1])
            self.get_logger().info(f'tray out {moved * 100:.1f} cm, {(tray[2] - tray0[2]) * 100:+.1f} cm in height')
        if key == 'lift' and vial and vial0 and p['task'] == 1:
            self.get_logger().info(f'vial lifted {(vial[2] - vial0[2]) * 100:.1f} cm out of the tray')
        if key == 'lift' and tray and tray0 and p['task'] == 2:
            self.get_logger().info(f'tray held at both ends, {(tray[2] - tray0[2]) * 100:.1f} cm above the board')


def main():
    rclpy.init()
    node = DemoRunner()
    try:
        node.run()
        node.wait(2.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
