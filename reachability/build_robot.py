#!/usr/bin/env python3
"""cuRobo robot config (collision spheres + self-collision matrix) from a URDF.

    build_robot.py robots/rby1m.urdf --assets <dir> --tools ee_left ee_right -o robots/rby1m.yml

Same steps as curobo.examples.getting_started.build_robot_model, without its
blanket error handler, so failures show a traceback.
"""
import argparse
import os
from pathlib import Path

from curobo.robot_builder import RobotBuilder


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('urdf')
    ap.add_argument('--assets', required=True, help='directory that package://<pkg>/... paths resolve against')
    ap.add_argument('--tools', nargs='+', required=True)
    ap.add_argument('--clip', nargs=3, action='append', metavar=('LINK', 'AXIS', 'OFFSET'), default=[])
    ap.add_argument('--samples', type=int, default=None, help='self-collision samples (builder default if unset)')
    ap.add_argument('--refit', nargs='*', default=[], metavar='LINK',
                    help='refit these links with denser (tighter) spheres, e.g. the hands that work in tight gaps')
    ap.add_argument('--refit-density', type=float, default=4.0)
    ap.add_argument('-o', '--output', required=True)
    a = ap.parse_args()

    b = RobotBuilder(urdf_path=a.urdf, asset_path=a.assets, tool_frames=a.tools)
    b.fit_collision_spheres(clip_links={lk: (ax, float(off)) for lk, ax, off in a.clip} or None)
    for link in a.refit:
        b.refit_link_spheres(link, sphere_density=a.refit_density)
    print(f'{b.num_spheres} spheres on {len(b.collision_link_names)} links'
          f"{f' ({len(a.refit)} refit at density {a.refit_density})' if a.refit else ''}")
    kw = {'num_samples': a.samples} if a.samples else {}
    b.compute_collision_matrix(prune_collisions=True, **kw)
    print(f'{len(b.collision_matrix)} self-collision ignore entries')
    b.save(b.build(), a.output)
    # paths relative to reachability/ (sweep.robot_config resolves them), not this machine's
    out = Path(a.output)
    here = Path(__file__).resolve().parent
    text = out.read_text()
    for key, path in (('urdf_path', Path(a.urdf)), ('asset_root_path', Path(a.assets))):
        text = text.replace(f'{key}: {path.resolve()}', f'{key}: {os.path.relpath(path.resolve(), here)}')
    out.write_text(text)
    print(f'wrote {a.output}')


if __name__ == '__main__':
    main()
