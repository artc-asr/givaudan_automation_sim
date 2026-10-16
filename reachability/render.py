#!/usr/bin/env python3
"""Picture of one solution: robot collision spheres, shelves and the tray, top + side view.

    env -u PYTHONPATH ../deps/curobo-venv/bin/python render.py results/rby1m_task2_fixed.json 5 [-o figures/x.png]
"""
import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402
import torch  # noqa: E402

import sweep  # noqa: E402

HERE = Path(__file__).resolve().parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('result')
    ap.add_argument('level', type=int)
    ap.add_argument('--problem', type=int, default=None, help='index; default: first successful')
    ap.add_argument('-o', '--out', default=None)
    a = ap.parse_args()

    from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
    from curobo.types import JointState

    d = json.load(open(a.result))
    lvl = d['levels'][a.level]
    probs = lvl['problems']
    i = a.problem if a.problem is not None else next((k for k, p in enumerate(probs) if p['success']), 0)
    pr = probs[i]
    rb = sweep.ROBOTS[d['robot']]
    config = sweep.robot_config(rb, d['task'], d['base'])
    ik = InverseKinematics(InverseKinematicsCfg.create(robot=config, num_seeds=1, self_collision_check=False))
    q = torch.tensor(pr['q'], device='cuda').view(1, -1)
    sph = ik.compute_kinematics(JointState.from_position(q, joint_names=d['joint_names'])).robot_spheres.view(-1, 4).cpu()
    links = sweep.sphere_links(config)

    gen = sweep.load_generator()
    boxes = sweep.world_boxes(gen, {sweep.Level(gen, a.level).id})
    tray_c, tray_d = pr['tray_box']
    fig, axes = plt.subplots(1, 2, figsize=(13, 6.5))
    for ax, (u, v, title) in zip(axes, ((0, 1, 'top view (x across the aisle, y along it)'),
                                        (0, 2, 'side view (x across the aisle, z up)'))):
        for name, (c, dims) in boxes.items():
            if name.endswith('/under') or (v == 1 and dims[2] < 0.05 and 'board' not in name):
                continue
            ax.add_patch(Rectangle((c[u] - dims[u] / 2, c[v] - dims[v] / 2), dims[u], dims[v],
                                   fc='#c9ced6', ec='#8a929e', lw=0.5, alpha=0.5 if v == 1 else 0.9))
        ax.add_patch(Rectangle((tray_c[u] - tray_d[u] / 2, tray_c[v] - tray_d[v] / 2), tray_d[u], tray_d[v],
                               fc='#d98a2b', ec='#8a4f0a', lw=1, alpha=0.9, zorder=3))
        order = sph[:, 2].argsort() if v == 1 else sph[:, 1].argsort()
        for k in order.tolist():
            x, y, z, r = sph[k].tolist()
            if r <= 0:
                continue
            hand = links[k] in rb['hand_links']
            ax.add_patch(Circle((x, (y, z)[v == 2]), r, fc='#3b6fb6' if not hand else '#2a9d55',
                                ec='white', lw=0.2, alpha=0.75, zorder=4))
        ax.set_aspect('equal')
        ax.set_xlim(-1.15, 1.15)
        ax.set_ylim((-0.9, 0.9) if v == 1 else (0, 2.25))
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.2)
    b = pr['base']
    fig.suptitle(f"{d['robot']} task {d['task']} base {d['base']}  level {a.level} (board {lvl['board_top']} m)  "
                 f"{pr['label']}  base x {b[0]:+.2f} y {b[1]:+.2f} yaw {b[2] * 57.3:+.0f} deg  "
                 f"tray clearance {pr['tray_clearance_mm']} mm  {'OK' if pr['success'] else 'FAIL'}", fontsize=10)
    out = Path(a.out or HERE / 'figures' / f"{Path(a.result).stem}_L{a.level:02d}.png")
    out.parent.mkdir(exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
