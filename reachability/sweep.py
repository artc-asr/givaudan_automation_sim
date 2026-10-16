#!/usr/bin/env python3
"""Highest shelf level a robot can work, per task, with cuRobo (both hands, whole body).

    env -u PYTHONPATH ../deps/curobo-venv/bin/python sweep.py --robot rby1m --task 1 --base mobile

Task 1: left hand holds the tray handle bar, tray pulled out PULL (back end still on
        the board: the datum); right hand = emulated suction tool pointing down on a
        vial in a row clear of the board above.
Task 2: the tray is fully out (back end CLEAR in front of the shelf face, lifted 5 mm);
        left hand on the handle bar (from the front or from above), right hand holds the
        back end from above, fingers either side of the tray (balance).
Base:   fixed = aisle centre facing shelf B (the Gazebo spawn); mobile = x, y, yaw free
        inside the aisle (virtual base joints, cuRobo extra_links).

Collision scene per level: the Gazebo world's boxes (shelf_world's generator) minus the
target tray, plus a keep-out box under each shelf (assumed: no driving under the bottom
board), plus the held tray in task 2 (minus the two hand zones).
"""
import argparse
import importlib.util
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import torch
import yaml

HERE = Path(__file__).resolve().parent
GEN = HERE.parent / 'src' / 'shelf_world' / 'scripts' / 'generate_world.py'

TARGET_SLOT = 10           # tray T10, the one at y = 0
SHELF = 'B'                # the shelf the robot works on (x > 0)
LIFT = 0.005               # task 2: tray lifted off the board level
RIGHT_YAWS = 8             # task 1: suction tool yaw about the vertical is free
RETURN_SEEDS = 16          # best solutions per problem; the first that passes the tray check wins

# Per robot: cuRobo config, base links for the mobile base, hand frames and offsets,
# finger locks per task (fingers must not be locked closed: their spheres overlap).
ROBOTS = {
    'rby1m': {
        'config': 'robots/rby1m.yml',
        'base_parent': 'base_footprint', 'base_child': 'base',
        'left': 'ee_left', 'right': 'ee_right',
        # links allowed to touch the tray they hold (excluded from the tray post-check)
        'hand_links': ['ee_left', 'ee_finger_l1', 'ee_finger_l2', 'link_left_arm_6',
                       'ee_right', 'ee_finger_r1', 'ee_finger_r2', 'link_right_arm_6'],
        'grip_depth': 0.10,        # hand frame -> centre of the finger pads, along -z
        'suction_len': 0.13,       # hand frame -> suction tip, along -z (task 1)
        'fixed_locks': {'wheel_fr': 0.0, 'wheel_fl': 0.0, 'wheel_rr': 0.0, 'wheel_rl': 0.0,
                        'head_0': 0.0, 'head_1': 0.0},
        'fingers': {1: {'gripper_finger_l1': -0.025, 'gripper_finger_l2': 0.025,     # 50 mm: 20 mm bar
                        'gripper_finger_r1': -0.025, 'gripper_finger_r2': 0.025},
                    2: {'gripper_finger_l1': -0.025, 'gripper_finger_l2': 0.025,
                        'gripper_finger_r1': -0.04, 'gripper_finger_r2': 0.04}},     # 80 mm: 58 mm tray
    },
}
BASE_LIMITS = {'x': (-0.45, 0.45), 'y': (-0.6, 0.6), 'yaw': (-math.pi, math.pi)}


# ------------------------------------------------------------------ geometry

def load_generator():
    spec = importlib.util.spec_from_file_location('generate_world', GEN)
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    return gen


def world_boxes(gen, exclude_ids):
    """Static collision boxes of the Gazebo world: {name: (centre xyz, dims xyz)}."""
    sdf, _ = gen.world_sdf(set(exclude_ids))
    world = ET.fromstring(sdf.split('\n', 1)[1]).find('world')
    boxes = {}
    for model in world.findall('model'):
        if model.get('name') in exclude_ids or model.findtext('static') != 'true':
            continue
        m = [float(v) for v in (model.findtext('pose') or '0 0 0').split()[:3]]
        for col in model.iter('collision'):
            size = col.find('geometry/box/size')
            if size is None:
                continue
            p = [float(v) for v in (col.findtext('pose') or '0 0 0').split()[:3]]
            boxes[f'{model.get("name")}/{col.get("name")}'] = (
                [m[i] + p[i] for i in range(3)], [float(v) for v in size.text.split()])
    for s, side in gen.SHELVES.items():          # keep-out under each shelf (assumption)
        cx = side * (gen.AISLE / 2 + gen.SHELF_D / 2)
        boxes[f'shelf_{s}/under'] = ([cx, 0, gen.BASE_Z / 2], [gen.SHELF_D, gen.SHELF_L, gen.BASE_Z])
    return boxes


def quat_wxyz(m):
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = [0.0] * 4
    q[0] = (m[k, j] - m[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (m[j, i] + m[i, j]) / s
    q[1 + k] = (m[k, i] + m[i, k]) / s
    return q


def frame(x_axis, z_axis):
    x, z = np.array(x_axis, float), np.array(z_axis, float)
    return np.column_stack([x, np.cross(z, x), z])


class Level:
    """Target tray geometry on one level of shelf B (world frame)."""

    def __init__(self, gen, level):
        z0, t = gen.board_list()[level]
        self.gen, self.level, self.top = gen, level, z0 + t
        self.side = gen.SHELVES[SHELF]
        self.out = -self.side                                   # x direction out of the shelf
        self.home_x = self.side * (gen.AISLE / 2 + gen.SHELF_D / 2)
        self.y = gen.tray_slots_y()[TARGET_SLOT - 1]
        self.face = self.side * gen.AISLE / 2
        self.id = f'tray_{SHELF}_L{level:02d}_T{TARGET_SLOT:02d}'

    def tray_point(self, tray_x, xt, yt, z):
        """Tray-frame (xt toward the handle, yt) -> world, tray centre at tray_x. Shelf B trays face -x."""
        return np.array([tray_x + self.out * xt, self.y - self.side * yt, z])

    def tray_box(self, tray_x, z):
        """The whole tray with its vials as one box (centre, dims), tray bottom at z."""
        g = self.gen
        h = g.TRAY_FLOOR + g.VIAL_H
        return self.tray_point(tray_x, 0, 0, z + h / 2).tolist(), [g.TRAY_L, g.TRAY_W, h]

    def bar(self, tray_x, z):
        g = self.gen
        return self.tray_point(tray_x, g.TRAY_L / 2 - g.HANDLE_ZONE / 2, 0, z + g.BAR_Z + g.BAR / 2)


def left_on_bar(bar, approach, depth):
    """Left hand: fingers point along `approach` (into the shelf), closing vertically, either way up."""
    return [(bar - approach * depth, quat_wxyz(frame([0, 0, s], -approach))) for s in (1, -1)]


def task1(lv, pull, rb):
    g = lv.gen
    tray_x = lv.home_x + lv.out * pull
    approach = np.array([-lv.out, 0, 0])
    left = left_on_bar(lv.bar(tray_x, lv.top), approach, rb['grip_depth'])
    problems = []
    xs = sorted({x for x, _ in g.vial_positions()}, reverse=True)       # row 1 nearest the handle
    for r, xt in enumerate(xs, start=1):
        vial = lv.tray_point(tray_x, xt, -g.COL_Y, lv.top + g.TRAY_FLOOR + g.VIAL_H)  # robot's right column
        if lv.side * vial[0] + g.VIAL_D / 2 >= lv.side * lv.face - 0.02:
            continue                                  # under the board above: not reachable from the top
        right = [(vial + np.array([0, 0, rb['suction_len']]),
                  quat_wxyz(frame([math.cos(a), math.sin(a), 0], [0, 0, 1])))
                 for a in (2 * math.pi * k / RIGHT_YAWS for k in range(RIGHT_YAWS))]
        problems.append({'label': f'row {r}', 'row': r, 'vial': vial.round(3).tolist(),
                         'tray_box': lv.tray_box(tray_x, lv.top),
                         'goals': [(lg, rg) for lg in left for rg in right]})
    return problems, {}


def task2(lv, rb, clear):
    g = lv.gen
    back_x = lv.face + lv.out * clear                     # back end this far in front of the face
    tray_x = back_x + lv.out * g.TRAY_L / 2
    z = lv.top + LIFT
    approach = np.array([-lv.out, 0, 0])
    bar = lv.bar(tray_x, z)
    # Grasp candidates, the solver picks: the tray is out in the aisle, so the robot stands
    # beside it and may hold the bar from the front (as when pulling) or from above
    # (fingers down, closing across the 20 mm bar, either way round).
    left = left_on_bar(bar, approach, rb['grip_depth']) + [
        (bar + np.array([0, 0, rb['grip_depth']]), quat_wxyz(frame([s, 0, 0], [0, 0, 1]))) for s in (1, -1)]
    grip = lv.tray_point(tray_x, -g.TRAY_L / 2 + 0.025, 0, z + g.TRAY_H / 2)   # 25 mm in from the back end
    right = [(grip + np.array([0, 0, rb['grip_depth']]), quat_wxyz(frame([0, s, 0], [0, 0, 1])))
             for s in (1, -1)]                            # fingers down, closing across the tray width
    # The held tray (body + vials) as an obstacle, minus 10 cm around each grasp: the hands'
    # collision spheres are ~5 cm, so a tighter cut makes the holding hands "collide" with it.
    # The middle ~0.3 m still keeps the body and arms from passing through the tray.
    # (Exact alternative, later: attach the tray to the hand as a cuRobo attached object.)
    x0, x1 = -g.TRAY_L / 2 + 0.025 + 0.10, g.TRAY_L / 2 - g.HANDLE_ZONE / 2 - 0.10
    c = lv.tray_point(tray_x, (x0 + x1) / 2, 0, z + (g.TRAY_FLOOR + g.VIAL_H) / 2)
    held = {'held_tray': (c.tolist(), [x1 - x0, g.TRAY_W, g.TRAY_FLOOR + g.VIAL_H])}
    return [{'label': 'full out', 'goals': [(lg, rg) for lg in left for rg in right],
             'grip': grip.round(3).tolist(), 'bar': lv.bar(tray_x, z).round(3).tolist(),
             'tray_box': lv.tray_box(tray_x, z)}], held


def box_clearance(spheres, centre, dims):
    """Signed distance of each sphere [N, 4] to an axis-aligned box (negative: overlap)."""
    q = (spheres[:, :3] - torch.tensor(centre)).abs() - torch.tensor(dims) / 2
    outside = q.clamp(min=0).norm(dim=-1)
    inside = q.max(dim=-1).values.clamp(max=0)
    return outside + inside - spheres[:, 3]


def sphere_links(config):
    """Link name of every collision sphere, in cuRobo's order (link by link, as in the config)."""
    k = config['kinematics']
    return [lk for lk in k['collision_link_names'] for _ in k['collision_spheres'].get(lk, [])]


# ------------------------------------------------------------------ robot model

def robot_config(rb, task, base):
    cfg = yaml.safe_load(open(HERE / rb['config']))
    k = cfg['kinematics']
    for key in ('urdf_path', 'asset_root_path'):          # stored relative to reachability/
        k[key] = str((HERE / k[key]).resolve())
    k['lock_joints'] = {**rb['fixed_locks'], **rb['fingers'][task]}
    if base == 'mobile':
        names = [('x', 'X_PRISM'), ('y', 'Y_PRISM'), ('yaw', 'Z_ROT')]
        parent = rb['base_parent']
        k['extra_links'] = {}
        for i, (n, jt) in enumerate(names):
            link = f'virtual_base_{n}'
            k['extra_links'][link] = {
                'parent_link_name': parent, 'link_name': link, 'joint_name': f'base_{n}',
                'joint_type': jt, 'fixed_transform': [0, 0, 0, 1, 0, 0, 0],
                'joint_limits': list(BASE_LIMITS[n]), 'joint_velocity_limits': [-1.0, 1.0],
                **({'child_link_name': rb['base_child']} if i == len(names) - 1 else {})}
            parent = link
        cs = k['cspace']
        n0 = len(cs['joint_names'])
        for key, v in cs.items():                 # one entry per new joint, like the first joint's
            if isinstance(v, list) and len(v) == n0:
                cs[key] = [f'base_{n}' for n, _ in names] + v if key == 'joint_names' else \
                    ([0.0] * 3 + v if key == 'default_joint_position' else [v[0]] * 3 + v)
    return cfg


# ------------------------------------------------------------------ solve

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', default='rby1m', choices=sorted(ROBOTS))
    ap.add_argument('--task', type=int, default=1, choices=(1, 2))
    ap.add_argument('--base', default='fixed', choices=('fixed', 'mobile'))
    ap.add_argument('--pull', type=float, default=0.25, help='task 1: how far the tray is pulled out [m]')
    ap.add_argument('--clear', type=float, default=0.10,
                    help='task 2: back end of the tray this far in front of the shelf face [m]')
    ap.add_argument('--seeds', type=int, default=None,
                    help='default: 256 for task 1 (1024 overflows Warp), 1024 for task 2 (256 misses its narrow solutions)')
    ap.add_argument('--scene', default='full', choices=('full', 'no-held', 'none'),
                    help='diagnosis: drop the held tray, or every obstacle')
    ap.add_argument('--drop', nargs='*', default=[], metavar='PREFIX',
                    help='diagnosis: remove obstacles whose name starts with PREFIX (e.g. shelf_A tray_)')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    a.seeds = a.seeds or (256 if a.task == 1 else 1024)
    out = Path(a.out or HERE / 'results' / f"{a.robot}_task{a.task}_{a.base}{'' if a.scene == 'full' else '_' + a.scene}.json")

    from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
    from curobo.scene import Cuboid, Scene
    from curobo.types import GoalToolPose, JointState

    rb = ROBOTS[a.robot]
    gen = load_generator()
    n_levels = len(gen.board_list())
    plans = []
    for lvl in range(n_levels):
        lv = Level(gen, lvl)
        problems, held = task1(lv, a.pull, rb) if a.task == 1 else task2(lv, rb, a.clear)
        boxes = {} if a.scene == 'none' else {**world_boxes(gen, {lv.id}), **(held if a.scene == 'full' else {})}
        boxes = {n: b for n, b in boxes.items() if not any(n.startswith(d) for d in a.drop)}
        plans.append((lv, problems, boxes))
    batch = max(len(p) for _, p, _ in plans)
    n_goal = max(len(pr['goals']) for _, p, _ in plans for pr in p)
    n_box = max(1, max(len(b) for *_, b in plans))

    config = robot_config(rb, a.task, a.base)
    ik = InverseKinematics(InverseKinematicsCfg.create(
        robot=config, scene_model={'cuboid': {}}, num_seeds=a.seeds,
        max_batch_size=batch, max_goalset=n_goal, self_collision_check=True,
        collision_cache={'cuboid': n_box + 4}))
    frames, names = ik.tool_frames, ik.joint_names
    links = sphere_links(config)
    body = torch.tensor([lk not in rb['hand_links'] for lk in links])   # spheres checked against the tray
    print(f'{a.robot} task {a.task} base {a.base}: {len(names)} joints, {batch} problems x {n_goal} goals per level, '
          f'up to {n_box} boxes')

    levels_out, t0 = [], time.time()
    for lv, problems, boxes in plans:
        ik.update_world(Scene(cuboid=[Cuboid(name=n, pose=[*c, 1, 0, 0, 0], dims=d) for n, (c, d) in boxes.items()]))
        pos = torch.zeros(batch, 1, len(frames), n_goal, 3)
        quat = torch.zeros(batch, 1, len(frames), n_goal, 4)
        padded = problems + [problems[-1]] * (batch - len(problems))
        for b, pr in enumerate(padded):
            goals = pr['goals'] + [pr['goals'][-1]] * (n_goal - len(pr['goals']))
            for gi, ((lp, lq), (rp, rq)) in enumerate(goals):
                for name, p, q in ((rb['left'], lp, lq), (rb['right'], rp, rq)):
                    i = frames.index(name)
                    pos[b, 0, i, gi] = torch.tensor(p)
                    quat[b, 0, i, gi] = torch.tensor(q)
        R = RETURN_SEEDS
        res = ik.solve_pose(GoalToolPose(tool_frames=frames, position=pos.cuda(), quaternion=quat.cuda()),
                            return_seeds=R)
        ok = res.success.view(batch, R).cpu()
        perr = res.position_error.view(batch, R, -1).cpu()
        rerr = res.rotation_error.view(batch, R, -1).cpu()
        js_names = list(res.js_solution.joint_names)                 # active joints + locked ones
        idx = torch.tensor([js_names.index(n) for n in names], device=res.js_solution.position.device)
        qs = res.js_solution.position.view(batch, R, -1)[..., idx]    # active joints, in `names` order
        # Post-check against the WHOLE tray the hands hold (the solver's scene leaves it out
        # or cut around the hands): every sphere except the hands' must clear it.
        spheres = ik.compute_kinematics(JointState.from_position(
            qs.reshape(batch * R, -1), joint_names=names)).robot_spheres.view(batch, R, -1, 4).cpu()
        if spheres.shape[2] != len(links):
            raise RuntimeError(f'{spheres.shape[2]} spheres but {len(links)} in the config order')
        rows = []
        for b, pr in enumerate(problems):
            clear = [float(box_clearance(spheres[b, k][body], *pr['tray_box']).min()) for k in range(R)]
            passing = [k for k in range(R) if ok[b, k] and clear[k] >= 0]
            k = passing[0] if passing else int(torch.argmax(torch.tensor(clear) * ok[b, :].float()
                                                            - 1e3 * (~ok[b, :]).float()))
            hits = sorted({links[i] for i in torch.nonzero(body).flatten().tolist()
                           if box_clearance(spheres[b, k][i:i + 1], *pr['tray_box'])[0] < 0})
            q = qs[b, k].cpu()
            qd = dict(zip(names, q.tolist()))
            rows.append({kk: v for kk, v in pr.items() if kk != 'goals'} | {
                'success': bool(passing), 'ik_success': bool(ok[b].any()),
                'tray_clearance_mm': round(clear[k] * 1000, 1), 'tray_hits': hits,
                'position_error_mm': round(float(perr[b, k].max()) * 1000, 1),
                'rotation_error': round(float(rerr[b, k].max()), 3), 'q': [round(v, 4) for v in q.tolist()],
                'base': [round(qd.get(f'base_{n}', 0.0), 3) for n in ('x', 'y', 'yaw')]})
        good = [r for r in rows if r['success']]
        best = min(rows, key=lambda r: (not r['success'], r['position_error_mm']))
        levels_out.append({'level': lv.level, 'board_top': round(lv.top, 3), 'success': bool(good), 'problems': rows})
        detail = (f"{', '.join(r['label'] for r in good)}  base x {best['base'][0]:+.2f} y {best['base'][1]:+.2f} "
                  f"yaw {math.degrees(best['base'][2]):+.0f} deg" if good else
                  (f"-  (IK ok but the body hits the tray: {', '.join(best['tray_hits'])})" if best['ik_success'] else
                   f"-  (closest: pos err {best['position_error_mm']:.1f} mm, rot err {best['rotation_error']:.3f})"))
        print(f'L{lv.level:02d} board {lv.top:.2f} m  OK: {detail}')
    dt = time.time() - t0
    best_level = max((d['level'] for d in levels_out if d['success']), default=None)
    print(f'highest level OK: {best_level}  ({dt:.1f} s)')
    out.parent.mkdir(exist_ok=True)
    json.dump({'robot': a.robot, 'task': a.task, 'base': a.base, 'scene': a.scene, 'pull': a.pull if a.task == 1 else None, 'clear': a.clear if a.task == 2 else None,
               'joint_names': names, 'highest_ok': best_level, 'solve_s': round(dt, 1), 'levels': levels_out},
              open(out, 'w'), indent=1)


if __name__ == '__main__':
    main()
