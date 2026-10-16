#!/usr/bin/env python3
"""Side view of a failed grasp: planned pose (real meshes), Gazebo's collision shapes at that
pose, and the pose Gazebo actually reached (from a recording).

    python3 illustrate_fail.py <run_dir> [--step grasp] [-o figures/x.png]

<run_dir> is what scripts/record_demo.sh wrote (recording.json, plan.json). Plain Python
(numpy, trimesh, matplotlib); the shelf comes from shelf_world's generator.
"""
import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
import numpy as np
import trimesh

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

import sweep  # noqa: E402
from align_axes import fk, origin_matrix  # noqa: E402

HERE = Path(__file__).resolve().parent
SHARE = HERE.parent / 'install' / 'roc_rby1' / 'share'
ARM = ['link_left_arm_3', 'link_left_arm_4', 'link_left_arm_5', 'link_left_arm_6', 'ee_left',
       'ee_finger_l1', 'ee_finger_l2']
BLAME = 'link_left_arm_5'


def base_matrix(b):
    m = np.eye(4)
    m[:3, :3] = [[math.cos(b['yaw']), -math.sin(b['yaw']), 0], [math.sin(b['yaw']), math.cos(b['yaw']), 0], [0, 0, 1]]
    m[:2, 3] = [b['x'], b['y']]
    return m


def shape_points(geom, dense=False):
    """Surface points of a URDF geometry in its own frame."""
    if geom.find('mesh') is not None:
        m = geom.find('mesh')
        mesh = trimesh.load(SHARE / m.get('filename').replace('package://', ''), force='mesh')
        pts = mesh.sample(4000) if dense else mesh.vertices
        return pts * np.array([float(v) for v in (m.get('scale') or '1 1 1').split()])
    if geom.find('box') is not None:
        return trimesh.creation.box(extents=[float(v) for v in geom.find('box').get('size').split()]).sample(1500)
    if geom.find('cylinder') is not None:
        c = geom.find('cylinder')
        return trimesh.creation.cylinder(radius=float(c.get('radius')), height=float(c.get('length'))).sample(2500)
    if geom.find('sphere') is not None:
        return trimesh.creation.icosphere(2, float(geom.find('sphere').get('radius'))).sample(1500)
    return np.zeros((0, 3))


def link_points(root, T, base, which):
    out = {}
    for link in root.findall('link'):
        n = link.get('name')
        if n not in ARM:
            continue
        pts = []
        for el in link.findall(which):
            g = el.find('geometry')
            M = base @ T[n] @ origin_matrix(el.find('origin'))
            p = shape_points(g, dense=True)
            pts.append(p @ M[:3, :3].T + M[:3, 3])
        if pts:
            out[n] = np.vstack(pts)
    return out


def inside(points, boxes):
    hit = np.zeros(len(points), bool)
    for c, d in boxes.values():
        hit |= np.all(np.abs(points - c) < np.array(d) / 2, axis=1)
    return hit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir', type=Path)
    ap.add_argument('--step', default='grasp')
    ap.add_argument('-o', '--out', type=Path, default=None)
    a = ap.parse_args()
    rec = json.loads((a.run_dir / 'recording.json').read_text())
    plan = json.loads((a.run_dir / 'plan.json').read_text())
    root = ET.parse(HERE / 'robots' / 'rby1m.urdf').getroot()
    base = base_matrix(plan['base'])
    step = next(s for s in plan['steps'] if s['phase'] == a.step)
    q_plan = dict(zip(plan['joint_names'], step['q'][-1]))
    q_plan.update(plan.get('fingers', {}))
    # measured: the last sample before the next step starts
    phases = [p['phase'] for p in rec['phases']]
    t_next = rec['phases'][phases.index(a.step) + 1]['t']
    i = max(0, int(np.searchsorted(rec['t'], t_next)) - 2)
    q_meas = dict(zip(rec['joint_names'], rec['joints'][i]))

    gen = sweep.load_generator()
    lv = sweep.Level(gen, plan['level'])
    boxes = {n: b for n, b in sweep.world_boxes(gen, {plan['tray']}).items() if not n.endswith('/under')}
    near = {n: b for n, b in boxes.items() if abs(b[0][1]) < 0.2 and abs(b[0][2] - lv.top) < 0.35}
    bar = lv.bar(lv.home_x, lv.top)

    panels = [('1. What cuRobo planned with: the real meshes at the planned grasp', q_plan, 'visual'),
              ("2. What Gazebo collides: rby1's collision shapes at the same planned pose", q_plan, 'collision'),
              ('3. What Gazebo reached: the arm stopped (measured joints, collision shapes)', q_meas, 'collision')]
    fig, axes = plt.subplots(1, 3, figsize=(18, 6.6), sharex=True, sharey=True)
    for ax, (title, q, which) in zip(axes, panels):
        for n, (c, d) in near.items():
            ax.add_patch(Rectangle((c[0] - d[0] / 2, c[2] - d[2] / 2), d[0], d[2], fc='#c9ced6', ec='#7d8592', lw=0.6,
                                   zorder=1))
        T = fk(root, q)
        pts = link_points(root, T, base, which)
        for n, p in pts.items():
            sl = np.abs(p[:, 1]) < 0.06                         # a slice through the handle (y ~ 0)
            p = p[sl]
            hit = inside(p, near)
            col = '#2a9d55' if n.startswith(('ee_', 'link_left_arm_6')) else '#3b6fb6'
            if n == BLAME and which == 'collision':
                col = '#d98a2b'
            ax.scatter(p[~hit, 0], p[~hit, 2], s=1.2, c=col, alpha=0.5, zorder=2, linewidths=0)
            if hit.any():
                ax.scatter(p[hit, 0], p[hit, 2], s=3, c='#c0392b', zorder=3, linewidths=0)
        ax.plot(bar[0], bar[2], marker='s', ms=7, mfc='none', mec='#111', zorder=4)
        ax.annotate('handle bar', (bar[0], bar[2]), (bar[0] + 0.03, bar[2] + 0.085), fontsize=9,
                    arrowprops={'arrowstyle': '->', 'lw': 0.8})
        ax.axvline(lv.face, color='#7d8592', lw=0.8, ls='--', zorder=0)
        ax.set_title(title, fontsize=10)
        ax.set_aspect('equal')
        ax.set_xlim(0.15, 0.75)
        ax.set_ylim(lv.top - 0.17, lv.top + 0.27)
        ax.set_xlabel('x [m] (into shelf B)')
        ax.grid(alpha=0.2)
    axes[0].set_ylabel('z [m]')
    # the numbers that explain it
    T = fk(root, q_plan)
    Tm = fk(root, q_meas)
    pad = lambda M: (base @ M['ee_left'])[:3, 3] + (base @ M['ee_left'])[:3, :3] @ [0, 0, -0.13]  # noqa: E731
    short = (pad(Tm) - pad(T)) * 1000
    cap = [c for c in root.find(f"link[@name='{BLAME}']").findall('collision') if c.find('geometry/cylinder') is not None][0]
    r = float(cap.find('geometry/cylinder').get('radius'))
    axes[1].text(0.02, 0.02, f'{BLAME} collision: capsule {2 * r * 1000:.0f} mm wide\n(real forearm ~110 mm); '
                 f'red = inside the shelf', transform=axes[1].transAxes, fontsize=9, va='bottom',
                 bbox={'fc': 'white', 'ec': '#ccc'})
    axes[2].text(0.02, 0.02, f'the capsule end rests on the shelf face:\nfingertips stopped {abs(short[0]):.0f} mm '
                 'short of the bar', transform=axes[2].transAxes, fontsize=9, va='bottom',
                 bbox={'fc': 'white', 'ec': '#ccc'})
    fig.suptitle(f"rby1, task 1, level {plan['level']} (board top {plan['board_top']:.2f} m): why the grasp fails in "
                 f"Gazebo. Board gap 100 mm; gripper body 126 mm tall. Slice |y| < 6 cm. "
                 f"Orange = {BLAME}, green = wrist + gripper.", fontsize=10)
    fig.tight_layout()
    out = a.out or HERE / 'figures' / f"rby1m_task{plan['task']}_L{plan['level']:02d}_{a.step}_fail.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f'wrote {out}; fingertips short by x {short[0]:+.0f} y {short[1]:+.0f} z {short[2]:+.0f} mm')


if __name__ == '__main__':
    main()
