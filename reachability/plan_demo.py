#!/usr/bin/env python3
"""Plan the Gazebo demo motions with cuRobo: timed joint trajectories per demo step.

    env -u PYTHONPATH ../deps/curobo-venv/bin/python plan_demo.py --task 1 [--level 8]
    env -u PYTHONPATH ../deps/curobo-venv/bin/python plan_demo.py --task 2 [--level 8]

Task 1 (base at the aisle centre facing shelf B, as spawned):
  reach   both hands to their ready poses (left 10 cm in front of the handle bar,
          right 12 cm above where the vial will be once the tray is out)
  grasp   left hand straight onto the bar               -> attach tray to ee_left
  pull    left hand pulls the tray PULL straight out    (back end stays on the board)
  suction right tool straight down onto the vial top    -> attach vial to ee_right
  lift    right hand lifts the vial 12 cm
Task 2 (base placed side-on to the tray path, chosen here, fixed during the task):
  reach   left 10 cm in front of the bar, right 15 cm above where the back end will be
  grasp   left hand onto the bar                        -> attach tray to ee_left
  pull    left hand pulls the tray 55 cm (its back end still on the board)
  support right hand up under the tray, 15 cm from the back end  -> attach tray to ee_right
  pull_out both hands carry it out until the back end is CLEAR in front of the shelf
  lift    both hands lift the tray 5 cm

Free moves: cuRobo MotionPlanner (collision-aware trajopt). Straight moves: a chain of
IK solves 1 cm apart, each from the previous pose (no joint jumps), with the tray
obstacle moved along. Writes plans/task<N>_L<level>.json for the demo node.
"""
import argparse
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import torch

import sweep

HERE = Path(__file__).resolve().parent
ROBOT = 'rby1m'
GROUPS = {'torso': [f'torso_{i}' for i in range(6)],          # the JTCs the demo node commands
          'left_arm': [f'left_arm_{i}' for i in range(7)],
          'right_arm': [f'right_arm_{i}' for i in range(7)]}
MAX_JOINT_VEL = 0.5        # rad/s, free moves and the joint-speed cap of straight moves
LINE_SPEED = {'grasp': 0.04, 'pull': 0.05, 'suction': 0.03, 'support': 0.03, 'lift': 0.03}   # m/s
SUPPORT_PULL = 0.55       # task 2: one-hand pull until the back end is 5 cm from the board edge
LINE_STEP = 0.01           # m between IK waypoints on straight moves
MAX_STEP_JUMP = 0.15       # rad: a bigger joint change between waypoints is a branch flip
# Margin for the right hand only: its finger cleared a board edge by 0.6 mm in the plan and
# caught it in Gazebo (task 2 reach; the spheres do not cover the parts exactly). The left
# hand's handle grasp has < 2 mm to spare (a 2 mm margin on all shelf boxes leaves no level),
# so a margin everywhere is not possible with the stock gripper.
MARGIN_LINKS = {'ee_right': 0.003, 'ee_finger_r1': 0.003, 'ee_finger_r2': 0.003}
FIRST_STEP_JUMP = 0.5      # rad, first waypoint of a straight move
LOCAL_SEEDS = 4            # seeds of the straight-move IK, all at the previous pose


# ------------------------------------------------------------------ frames

def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def quat_to_mat(q):            # w x y z
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Base:
    """The robot's fixed base pose in the world; maps world targets / boxes into its frame."""

    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
        self.r_inv = rotz(-yaw)

    def point(self, p):
        return self.r_inv @ (np.asarray(p) - [self.x, self.y, 0])

    def pose(self, p, q):
        return self.point(p), sweep.quat_wxyz(self.r_inv @ quat_to_mat(q))

    def scene(self, boxes):
        from curobo.scene import Cuboid, Scene
        q = sweep.quat_wxyz(self.r_inv)
        return Scene(cuboid=[Cuboid(name=n, pose=[*self.point(c), *q], dims=d) for n, (c, d) in boxes.items()])


# ------------------------------------------------------------------ demo geometry

class Demo:
    """Hand targets (world frame) for one level and task."""

    def __init__(self, gen, level, task, pull, clear):
        self.gen, self.task = gen, task
        self.lv = sweep.Level(gen, level)
        lv, g = self.lv, gen
        self.rb = sweep.ROBOTS[ROBOT]
        self.approach = np.array([-lv.out, 0.0, 0.0])           # into the shelf
        self.home_x = lv.home_x
        if task == 1:
            self.pull = pull
        else:
            back_x = lv.face + lv.out * clear
            self.pull = abs((back_x + lv.out * g.TRAY_L / 2) - lv.home_x)
        self.depth = self.rb['grip_depth']

    def tray_x(self, pulled):
        return self.home_x + self.lv.out * pulled

    def left(self, pulled, s, extra=0.0, lift=0.0):
        """Left hand on the bar (fingers into the shelf, closing vertically: s = +-1)."""
        bar = self.lv.bar(self.tray_x(pulled), self.lv.top) + [0, 0, lift]
        return bar - self.approach * (self.rb['bar_depth'] + extra), sweep.quat_wxyz(sweep.frame([0, 0, s], -self.approach))

    def vial(self, pulled):
        g = self.gen
        # row 3, robot's right column (tray-frame +y on shelf B): the generator's --pick-vial 3:1
        xt, yt = g.vial_xy(3, 1)
        return self.lv.tray_point(self.tray_x(pulled), xt, yt, self.lv.top + g.TRAY_FLOOR + g.VIAL_H)

    def suction(self, pulled, yaw, up=0.0):
        """Right tool tip on the vial top (pointing down), tool yaw `yaw`."""
        tip = self.vial(pulled) + [0, 0, self.rb['suction_len'] + up]
        return tip, sweep.quat_wxyz(sweep.frame([math.cos(yaw), math.sin(yaw), 0], [0, 0, 1]))

    def back_grip(self, pulled, s, up=0.0):
        """Right hand from above on the back end, fingers either side of the tray (s = +-1)."""
        g = self.gen
        grip = self.lv.tray_point(self.tray_x(pulled), -g.TRAY_L / 2 + 0.025, 0, self.lv.top + g.TRAY_H / 2 + up)
        return grip + [0, 0, self.depth], sweep.quat_wxyz(sweep.frame([0, s, 0], [0, 0, 1]))

    def support_xt(self):
        """Tray-frame x of the right hand's support point: 15 cm in from the back end."""
        return -self.gen.TRAY_L / 2 + 0.15

    def under(self, pulled, s, down=0.0, lift=0.0):
        """Right hand under the tray at the support point, fingers up either side of it (s = +-1)."""
        p = self.lv.tray_point(self.tray_x(pulled), self.support_xt(), 0, self.lv.top + lift)
        return p - np.array([0, 0, self.depth + down]), sweep.quat_wxyz(sweep.frame([0, s, 0], [0, 0, -1]))

    def tray_obstacle(self, pulled, lift=0.0):
        """The target tray at its current pose, minus 10 cm around each hand zone. Task 1: the
        tray body only (the suction tool is meant to touch a vial top); task 2: body + vials."""
        g = self.gen
        h = g.TRAY_H if self.task == 1 else g.TRAY_FLOOR + g.VIAL_H
        x1 = g.TRAY_L / 2 - g.HANDLE_ZONE / 2 - 0.10
        x0 = -g.TRAY_L / 2 + 0.125 if self.task == 1 else self.support_xt() + 0.08
        c = self.lv.tray_point(self.tray_x(pulled), (x0 + x1) / 2, 0, self.lv.top + lift + h / 2)
        return {'target_tray': (c.tolist(), [x1 - x0, g.TRAY_W, h])}

    def scene(self, pulled, lift=0.0):
        return {**sweep.world_boxes(self.gen, {self.lv.id}), **self.tray_obstacle(pulled, lift)}


# ------------------------------------------------------------------ solvers

def initial_q(names):
    """Gazebo start pose: the ros2_control initial values in the URDF (0 where none)."""
    root = ET.parse(HERE / 'robots' / 'rby1m.urdf').getroot()
    q0 = {}
    for j in root.iter('joint'):
        for p in j.iter('param'):
            if p.get('name') == 'initial_value' and j.get('name'):
                q0[j.get('name')] = float(p.text)
    return np.array([q0.get(n, 0.0) for n in names])


class Planner:
    def __init__(self, task, seeds):
        from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
        from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
        cfg = sweep.robot_config(sweep.ROBOTS[ROBOT], task, 'fixed')
        for link, extra in MARGIN_LINKS.items():
            for sph in cfg['kinematics']['collision_spheres'][link]:
                sph['radius'] += extra
        cache = {'cuboid': 128}
        self.ik = InverseKinematics(InverseKinematicsCfg.create(
            robot=cfg, scene_model={'cuboid': {}}, num_seeds=seeds, max_batch_size=1, max_goalset=16,
            self_collision_check=True, collision_cache=cache))
        # straight moves: a few seeds, all at the previous pose, so the solution stays on the
        # same arm branch (with 128 random seeds the best one often flips the arm: 1-6 rad jumps)
        # gradient stage only: the particle stage spreads the seeds out on purpose
        self.ik_local = InverseKinematics(InverseKinematicsCfg.create(
            robot=cfg, optimizer_configs=['ik/lbfgs_ik.yml'], scene_model={'cuboid': {}}, num_seeds=LOCAL_SEEDS, max_batch_size=1, max_goalset=16,
            self_collision_check=True, collision_cache=cache))
        # and no LM seed stage: it would replace our seeds with its own 32 (use_lm_seed)
        self.ik_local.config.use_lm_seed = False
        self.mp = MotionPlanner(MotionPlannerCfg.create(
            robot=cfg, scene_model={'cuboid': {}}, collision_cache=cache, max_goalset=1))
        self.mp.warmup(enable_graph=True, num_warmup_iterations=2)
        self.names = self.ik.joint_names
        self.frames = self.ik.tool_frames
        assert self.names == self.mp.joint_names, 'IK and planner joint order differ'

    def goal(self, targets):
        """targets: {frame: [(pos, quat), ...]} in the base frame -> GoalToolPose (goalset = list index)."""
        from curobo.types import GoalToolPose
        n = max(len(v) for v in targets.values())
        pos = torch.zeros(1, 1, len(self.frames), n, 3)
        quat = torch.zeros(1, 1, len(self.frames), n, 4)
        for f, goals in targets.items():
            goals = goals + [goals[-1]] * (n - len(goals))
            for k, (p, q) in enumerate(goals):
                pos[0, 0, self.frames.index(f), k] = torch.tensor(np.asarray(p, float))
                quat[0, 0, self.frames.index(f), k] = torch.tensor(np.asarray(q, float))
        return GoalToolPose(tool_frames=list(self.frames), position=pos.cuda(), quaternion=quat.cuda())

    def solve(self, base, scene, targets, q_seed, local=False):
        """IK near q_seed (local: every seed starts at q_seed). Returns (q, goal index, pos err m)."""
        from curobo.types import JointState
        ik = self.ik_local if local else self.ik
        ik.update_world(base.scene(scene))
        q_t = torch.tensor(q_seed, dtype=torch.float32).cuda().view(1, -1)
        cur = JointState.from_position(q_t, joint_names=self.names)
        seeds = q_t.view(1, 1, -1).repeat(1, LOCAL_SEEDS, 1) if local else None
        res = ik.solve_pose(self.goal({f: [base.pose(p, q) for p, q in v] for f, v in targets.items()}),
                            current_state=cur, seed_config=seeds)
        names = list(res.js_solution.joint_names)
        q = res.js_solution.position.view(-1, len(names))[0].cpu().numpy()
        q = np.array([q[names.index(n)] for n in self.names])
        gi = int(res.goalset_index.view(-1)[0]) if res.goalset_index is not None else 0
        err = float(res.position_error.max())
        return (q if bool(res.success.view(-1)[0]) else None), gi, err

    def solve_nearest(self, base, scene, targets, q_prev, n=16):
        """Fallback for a straight-move waypoint: the normal solver's n best solutions, the
        successful one closest to q_prev. Returns (q or None, its max joint change)."""
        from curobo.types import JointState
        self.ik.update_world(base.scene(scene))
        cur = JointState.from_position(torch.tensor(q_prev, dtype=torch.float32).cuda().view(1, -1),
                                       joint_names=self.names)
        res = self.ik.solve_pose(self.goal({f: [base.pose(p, q) for p, q in v] for f, v in targets.items()}),
                                 current_state=cur, return_seeds=n)
        names = list(res.js_solution.joint_names)
        idx = [names.index(j) for j in self.names]
        qs = res.js_solution.position.view(n, len(names)).cpu().numpy()[:, idx]
        ok = res.success.view(n).cpu().numpy()
        best = None
        for q, good in zip(qs, ok):
            if good and (best is None or np.abs(q - q_prev).max() < np.abs(best - q_prev).max()):
                best = q
        return best, (float(np.abs(best - q_prev).max()) if best is not None else float('inf'))

    def move(self, base, scene, targets, q_start):
        """Collision-free trajectory from q_start to the targets: [T, dof] and dt."""
        from curobo.types import JointState
        self.mp.update_world(base.scene(scene))
        cur = JointState.from_position(torch.tensor(q_start, dtype=torch.float32).cuda().view(1, -1),
                                       joint_names=self.names)
        res = self.mp.plan_pose(self.goal({f: [base.pose(*v[0])] for f, v in targets.items()}), cur)
        if res is None or not bool(res.success.any()):
            return None
        plan = res.get_interpolated_plan()
        names = list(plan.joint_names)
        pos = plan.position.view(-1, len(names)).cpu().numpy()
        idx = [names.index(n) for n in self.names]
        return pos[:, idx]


def chain(planner, base, steps, q_start):
    """IK along waypoints: steps = [(scene, targets)]; each from the previous solution."""
    out, q = [q_start], q_start
    for scene, targets in steps:
        q_new, _, err = planner.solve(base, scene, targets, q, local=True)
        if q_new is None:                      # local solver stuck: nearest of the global solutions
            q_new, _ = planner.solve_nearest(base, scene, targets, q)
        if q_new is None:
            return None, f'IK failed (pos err {err * 1000:.1f} mm)'
        d = np.abs(q_new - q)
        # the first waypoint also absorbs the previous move's end tolerance (5 mm, 0.05 rad)
        if d.max() > (FIRST_STEP_JUMP if len(out) == 1 else MAX_STEP_JUMP):
            return None, f'joint jump {d.max():.2f} rad ({planner.names[int(d.argmax())]}) at waypoint {len(out)}'
        out.append(q_new)
        q = q_new
    return np.array(out), ''


def timed(path, min_dt, label):
    """Times for a joint path: each segment at most MAX_JOINT_VEL per joint and at least min_dt."""
    t = [0.0]
    for a, b in zip(path[:-1], path[1:]):
        t.append(t[-1] + max(min_dt, float(np.abs(b - a).max()) / MAX_JOINT_VEL))
    return t


def line(a, b, step=LINE_STEP):
    n = max(1, int(math.ceil(np.linalg.norm(np.asarray(b) - np.asarray(a)) / step)))
    return [np.asarray(a) + (np.asarray(b) - np.asarray(a)) * k / n for k in range(1, n + 1)]


# ------------------------------------------------------------------ plans

def plan_task(planner, demo, base, choice):
    """The demo steps for one base pose and hand-orientation choice; raises on failure."""
    q = initial_q(planner.names)
    steps = []
    s, yaw = choice['left_flip'], choice.get('right', 0.0)          # yaw: task 1's suction tool

    def add(key, label, desc, path, speed=None, events=(), check=None):
        """check: (hand frame, its planned world position) the runner verifies before the events."""
        if speed is None:                              # free move: joint-speed limited
            times = timed(path, 0.02, key)
        else:                                          # straight move: LINE_STEP at `speed`, joint cap
            times = timed(path, LINE_STEP / speed, key)
        steps.append({'phase': key, 'label': label, 'desc': desc, 'events': list(events),
                      **({'check': {'frame': check[0], 'pos': [round(float(v), 4) for v in check[1]]}} if check else {}),
                      'times': [round(v, 3) for v in times], 'q': [[round(float(v), 5) for v in p] for p in path]})
        return path[-1]

    L, R = planner.rb_left, planner.rb_right
    if demo.task == 1:
        pull = demo.pull
        right_ready = lambda up: demo.suction(pull, yaw, up)               # noqa: E731
        path = planner.move(base, demo.scene(0.0), {L: [demo.left(0.0, s, extra=0.10)], R: [right_ready(0.12)]}, q)
        if path is None:
            raise RuntimeError('reach: no collision-free plan')
        q = add('reach', 'Reach the tray', 'Left hand to 10 cm in front of the handle bar; right '
                'suction tool to 12 cm above the vial it will pick.', path)
        wp = [(demo.scene(0.0), {L: [(p, demo.left(0.0, s)[1])], R: [right_ready(0.12)]})
              for p in line(demo.left(0.0, s, extra=0.10)[0], demo.left(0.0, s)[0])]
        path, why = chain(planner, base, wp, q)
        if path is None:
            raise RuntimeError(f'grasp: {why}')
        q = add('grasp', 'Grasp the handle', 'Straight onto the bar; the fingertips pinch it.', path,
                LINE_SPEED['grasp'], ['attach tray ee_left'], (L, demo.left(0.0, s)[0]))
        n = int(math.ceil(pull / LINE_STEP))
        wp = [(demo.scene(pull * k / n), {L: [demo.left(pull * k / n, s)], R: [right_ready(0.12)]})
              for k in range(1, n + 1)]
        path, why = chain(planner, base, wp, q)
        if path is None:
            raise RuntimeError(f'pull: {why}')
        q = add('pull', f'Pull the tray {pull * 100:.0f} cm', 'Straight out of the shelf; the back end stays '
                'on the board as the datum.', path, LINE_SPEED['pull'])
        wp = [(demo.scene(pull), {L: [demo.left(pull, s)], R: [(p, right_ready(0)[1])]})
              for p in line(right_ready(0.12)[0], right_ready(0.0)[0])]
        path, why = chain(planner, base, wp, q)
        if path is None:
            raise RuntimeError(f'suction: {why}')
        q = add('suction', 'Suction onto a vial', 'The right tool comes straight down onto row 3; '
                'suction on.', path, LINE_SPEED['suction'], ['attach vial ee_right'], (R, right_ready(0.0)[0]))
        wp = [(demo.scene(pull), {L: [demo.left(pull, s)], R: [(p, right_ready(0)[1])]})
              for p in line(right_ready(0.0)[0], right_ready(0.12)[0])]
        path, why = chain(planner, base, wp, q)
        if path is None:
            raise RuntimeError(f'lift: {why}')
        add('lift', 'Lift the vial', 'Straight up 12 cm, clear of its neighbours.', path, LINE_SPEED['lift'])
    else:
        # The left hand alone cannot hold the full tray once its back end leaves the board
        # (first run: it dropped 17 cm and tilted 20 deg at 60 cm). So: pull until the back end
        # is 5 cm from the board edge, the right hand comes up under the tray just outside the
        # shelf, then both hands pull it out together and lift it.
        pull, p1, rf = demo.pull, SUPPORT_PULL, choice['right_flip']
        wait = demo.under(p1, rf, down=0.15)
        path = planner.move(base, demo.scene(0.0), {L: [demo.left(0.0, s, extra=0.10)], R: [wait]}, q)
        if path is None:
            raise RuntimeError('reach: no collision-free plan')
        q = add('reach', 'Reach the tray', 'Left hand to 10 cm in front of the handle bar; right hand '
                '15 cm under where it will support the tray.', path)
        wp = [(demo.scene(0.0), {L: [(p, demo.left(0.0, s)[1])], R: [wait]})
              for p in line(demo.left(0.0, s, extra=0.10)[0], demo.left(0.0, s)[0])]
        path, why = chain(planner, base, wp, q)
        if path is None:
            raise RuntimeError(f'grasp: {why}')
        q = add('grasp', 'Grasp the handle', 'Straight onto the bar; the fingertips pinch it.', path,
                LINE_SPEED['grasp'], ['attach tray ee_left'], (L, demo.left(0.0, s)[0]))
        n = int(math.ceil(p1 / LINE_STEP))
        wp = [(demo.scene(p1 * k / n), {L: [demo.left(p1 * k / n, s)], R: [wait]}) for k in range(1, n + 1)]
        path, why = chain(planner, base, wp, q)
        if path is None:
            raise RuntimeError(f'pull: {why}')
        q = add('pull', f'Pull the tray {p1 * 100:.0f} cm', 'One hand, while the back end still rests on the '
                'board.', path, LINE_SPEED['pull'])
        wp = [(demo.scene(p1), {L: [demo.left(p1, s)], R: [(p, wait[1])]})
              for p in line(wait[0], demo.under(p1, rf)[0])]
        path, why = chain(planner, base, wp, q)
        if path is None:
            raise RuntimeError(f'support: {why}')
        q = add('support', 'Support it from below', 'Right hand up under the tray, fingers either side, '
                '15 cm from the back end, before the tray leaves the board.', path, LINE_SPEED['support'],
                ['attach tray ee_right'], (R, demo.under(p1, rf)[0]))
        n = int(math.ceil((pull - p1) / LINE_STEP))
        wp = [(demo.scene(p1 + (pull - p1) * k / n), {L: [demo.left(p1 + (pull - p1) * k / n, s)],
                                                      R: [demo.under(p1 + (pull - p1) * k / n, rf)]})
              for k in range(1, n + 1)]
        path, why = chain(planner, base, wp, q)
        if path is None:
            raise RuntimeError(f'pull out: {why}')
        q = add('pull_out', f'Take it out together ({(pull - p1) * 100:.0f} cm)', 'Both hands carry the tray '
                'until its back end is clear of the shelf.', path, LINE_SPEED['pull'])
        k = 5
        wp = [(demo.scene(pull, lift=0.05 * i / k), {L: [demo.left(pull, s, lift=0.05 * i / k)],
                                                    R: [demo.under(pull, rf, lift=0.05 * i / k)]}) for i in range(1, k + 1)]
        path, why = chain(planner, base, wp, q)
        if path is None:
            raise RuntimeError(f'lift: {why}')
        add('lift', 'Lift the tray', 'Both hands lift the tray 5 cm, level.', path, LINE_SPEED['lift'])
    return steps


def candidates(task):
    """(x, y, yaw) base poses to try, the base fixed for the whole task (no driving with the
    tray in hand). rby1's base is 0.76 m long: facing the shelf its centre may come to
    x = 0.12 before it touches the shelf face (0.50). Task 1 faces the shelf; task 2 also
    tries side-on poses (the fully pulled tray reaches past the aisle centre)."""
    facing = [(x, y, math.radians(a)) for x in (0.12, 0.08, 0.04, 0.0) for y in (0.0, -0.1, 0.1)
              for a in (0, -15, 15)]
    if task == 1:
        return facing
    side = [(x, y, math.radians(a)) for y in (-0.45, -0.35, 0.35, 0.45) for x in (-0.05, 0.1, 0.25)
            for a in ((90, 60, 120) if y < 0 else (-90, -60, -120))]
    return side + facing


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--task', type=int, choices=(1, 2), required=True)
    ap.add_argument('--level', type=int, default=None, help='default: the highest level that plans end to end')
    ap.add_argument('--pull', type=float, default=0.25, help='task 1 [m]')
    ap.add_argument('--clear', type=float, default=0.10, help='task 2: back end this far in front of the shelf [m]')
    ap.add_argument('--seeds', type=int, default=128)
    a = ap.parse_args()

    gen = sweep.load_generator()
    planner = Planner(a.task, a.seeds)
    rb = sweep.ROBOTS[ROBOT]
    planner.rb_left, planner.rb_right = rb['left'], rb['right']
    choices = ([{'left_flip': s, 'right': 2 * math.pi * k / 8} for s in (1, -1) for k in range(8)] if a.task == 1 else
               [{'left_flip': s, 'right_flip': r} for s in (1, -1) for r in (1, -1)])
    q0 = initial_q(planner.names)
    t0 = time.time()
    plan, tried = None, 0
    levels = [a.level] if a.level is not None else list(reversed(range(len(gen.board_list()))))
    for level in levels:
        demo = Demo(gen, level, a.task, a.pull, a.clear)
        plan, tried = find_plan(planner, demo, a, rb, choices, q0, gen)
        print(f'level {level} (board {demo.lv.top:.2f} m): {"planned" if plan else "no plan"} '
              f'({tried} base/hand candidates passed the key-pose check, {time.time() - t0:.0f} s so far)')
        if plan:
            break
    if plan is None:
        raise SystemExit('no level could be planned')
    write(plan, a, t0)


def key_targets(demo, rb, ch):
    """(grasp key pose, end key pose) targets for one hand choice."""
    L, R = rb['left'], rb['right']
    if demo.task == 1:
        return ({L: [demo.left(0.0, ch['left_flip'])], R: [demo.suction(demo.pull, ch['right'], 0.12)]},
                {L: [demo.left(demo.pull, ch['left_flip'])], R: [demo.suction(demo.pull, ch['right'])]})
    return ({L: [demo.left(0.0, ch['left_flip'])], R: [demo.under(SUPPORT_PULL, ch['right_flip'], down=0.15)]},
            {L: [demo.left(demo.pull, ch['left_flip'], lift=0.05)], R: [demo.under(demo.pull, ch['right_flip'], lift=0.05)]})


def find_plan(planner, demo, a, rb, choices, q0, gen):
    """First (base pose, hand choice) whose whole demo plans; (plan or None, candidates tried)."""
    plan, tried = None, 0
    for x, y, yaw in candidates(a.task):
        base = Base(x, y, yaw)
        # Key-pose pre-filter, batched: all hand choices as one goalset (cuRobo returns the
        # nearest feasible one) at the end of the pull, then that choice at the grasp.
        key_end = {rb['left']: [], rb['right']: []}
        for ch in choices:
            for f, target in key_targets(demo, rb, ch)[1].items():
                key_end[f] += target
        q_end, gi, _ = planner.solve(base, demo.scene(demo.pull), key_end, q0)
        if q_end is None:
            continue
        ch = choices[gi]
        if planner.solve(base, demo.scene(0.0), key_targets(demo, rb, ch)[0], q0)[0] is None:
            continue
        for ch in [ch]:
            tried += 1
            try:
                steps = plan_task(planner, demo, base, ch)
            except RuntimeError as e:
                print(f'base ({x:+.2f}, {y:+.2f}, {math.degrees(yaw):+.0f} deg) {ch}: {e}', flush=True)
                continue
            plan = {'robot': ROBOT, 'task': a.task, 'level': demo.lv.level, 'board_top': round(demo.lv.top, 3),
                    'tray': demo.lv.id, 'vial': f"vial_{demo.lv.id[5:]}_r03c1", 'pull': round(demo.pull, 3),
                    'base': {'x': x, 'y': y, 'yaw': yaw}, 'choice': ch, 'joint_names': planner.names,
                    'groups': GROUPS, 'start_q': [round(float(v), 5) for v in q0], 'steps': steps,
                    # finger positions the plan's collision model assumed (the runner sets them)
                    'fingers': sweep.ROBOTS[ROBOT]['fingers'][a.task],
                    'finger_groups': {'left_gripper': ['gripper_finger_l1', 'gripper_finger_l2'],
                                      'right_gripper': ['gripper_finger_r1', 'gripper_finger_r2']},
                    'tray_home': {'x': demo.home_x, 'y': demo.lv.y, 'z': demo.lv.top + 0.001,
                                  'yaw': 0.0 if demo.lv.side < 0 else math.pi},
                    # as the generator places them: tray 1 mm above the board, vial 0.5 mm above the tray floor
                    'vial_home': [round(float(v), 4) for v in
                                  (demo.vial(0.0) + [0, 0, 0.001 + 0.0005 - gen.VIAL_H])]}
            return plan, tried
    return None, tried


def write(plan, a, t0):
    out = HERE / 'plans' / f"task{a.task}_L{plan['level']:02d}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(plan, indent=1))
    dur = sum(s['times'][-1] for s in plan['steps'])
    b = plan['base']
    print(f"wrote {out}: base ({b['x']:+.2f}, {b['y']:+.2f}, {math.degrees(b['yaw']):+.0f} deg), "
          f"{len(plan['steps'])} steps, {dur:.0f} s of motion, planned in {time.time() - t0:.0f} s")
    for s in plan['steps']:
        print(f"  {s['phase']:8s} {len(s['q']):4d} pts {s['times'][-1]:6.1f} s  {' '.join(s['events'])}")


if __name__ == '__main__':
    main()
