#!/usr/bin/env python3
"""cuRobo robot config (collision spheres + self-collision matrix) from a URDF.

    build_robot.py robots/rby1m.urdf --assets <dir> --tools ee_left ee_right -o robots/rby1m.yml

Same steps as curobo.examples.getting_started.build_robot_model, without its
blanket error handler, so failures show a traceback.
"""
import argparse

from curobo.robot_builder import RobotBuilder


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('urdf')
    ap.add_argument('--assets', required=True, help='directory that package://<pkg>/... paths resolve against')
    ap.add_argument('--tools', nargs='+', required=True)
    ap.add_argument('--clip', nargs=3, action='append', metavar=('LINK', 'AXIS', 'OFFSET'), default=[])
    ap.add_argument('--samples', type=int, default=None, help='self-collision samples (builder default if unset)')
    ap.add_argument('-o', '--output', required=True)
    a = ap.parse_args()

    b = RobotBuilder(urdf_path=a.urdf, asset_path=a.assets, tool_frames=a.tools)
    b.fit_collision_spheres(clip_links={lk: (ax, float(off)) for lk, ax, off in a.clip} or None)
    print(f'{b.num_spheres} spheres on {len(b.collision_link_names)} links')
    kw = {'num_samples': a.samples} if a.samples else {}
    b.compute_collision_matrix(prune_collisions=True, **kw)
    print(f'{len(b.collision_matrix)} self-collision ignore entries')
    b.save(b.build(), a.output)
    print(f'wrote {a.output}')


if __name__ == '__main__':
    main()
