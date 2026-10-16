#!/usr/bin/env python3
"""World-frame collision extent of a URDF at its initial joint positions, per link.

    xacro robot.urdf.xacro | scripts/urdf_extent.py [--x 0 --y 0 --yaw 0] [--top 8]

Joint positions: ros2_control <param name="initial_value"> where given, else 0.
Meshes are loaded with trimesh (package:// resolved through ament). Prints the
whole-robot box and the links that reach furthest in x (across the aisle).
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np


def rpy_xyz(el):
    if el is None:
        return np.eye(4)
    xyz = [float(v) for v in el.get('xyz', '0 0 0').split()]
    r, p, y = (float(v) for v in el.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    t = np.eye(4)
    t[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    t[:3, 3] = xyz
    return t


def axis_motion(jtype, axis, q):
    t = np.eye(4)
    a = np.array(axis) / np.linalg.norm(axis)
    if jtype in ('revolute', 'continuous'):
        k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        t[:3, :3] = np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * k @ k
    elif jtype == 'prismatic':
        t[:3, 3] = a * q
    return t


def resolve(uri):
    if uri.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        pkg, rest = uri[len('package://'):].split('/', 1)
        return f'{get_package_share_directory(pkg)}/{rest}'
    return uri[len('file://'):] if uri.startswith('file://') else uri


def geometry_points(geom):
    g = list(geom)[0]
    if g.tag == 'box':
        sx, sy, sz = (float(v) / 2 for v in g.get('size').split())
        return np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    if g.tag == 'cylinder':
        r, h = float(g.get('radius')), float(g.get('length')) / 2
        return np.array([[x, y, z] for x in (-r, r) for y in (-r, r) for z in (-h, h)])
    if g.tag == 'sphere':
        r = float(g.get('radius'))
        return np.array([[x, y, z] for x in (-r, r) for y in (-r, r) for z in (-r, r)])
    if g.tag == 'mesh':
        import trimesh
        m = trimesh.load(resolve(g.get('filename')), force='mesh')
        scale = np.array([float(v) for v in g.get('scale', '1 1 1').split()])
        return m.bounding_box.vertices * scale
    return np.zeros((0, 3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=0.0)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--z', type=float, default=0.0)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--top', type=int, default=8)
    a = ap.parse_args()
    root = ET.fromstring(sys.stdin.read())

    q0 = {}
    for j in root.iter('joint'):
        for p in j.iter('param'):
            if p.get('name') == 'initial_value' and j.get('name'):
                q0[j.get('name')] = float(p.text)
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    links = {lk.get('name'): lk for lk in root.findall('link')}
    rootlink = next(n for n in links if n not in joints)

    base = rpy_xyz(ET.Element('o', xyz=f'{a.x} {a.y} {a.z}', rpy=f'0 0 {a.yaw}'))
    cache = {rootlink: base}

    def pose(name):
        if name not in cache:
            j = joints[name]
            axis = [float(v) for v in (j.find('axis').get('xyz') if j.find('axis') is not None else '1 0 0').split()]
            cache[name] = (pose(j.find('parent').get('link')) @ rpy_xyz(j.find('origin'))
                           @ axis_motion(j.get('type'), axis, q0.get(j.get('name'), 0.0)))
        return cache[name]

    rows, allpts = [], []
    for name, lk in links.items():
        pts = []
        for c in lk.findall('collision'):
            local = geometry_points(c.find('geometry'))
            if len(local):
                h = np.c_[local, np.ones(len(local))] @ (pose(name) @ rpy_xyz(c.find('origin'))).T
                pts.append(h[:, :3])
        if pts:
            p = np.vstack(pts)
            allpts.append(p)
            rows.append((name, p.min(0), p.max(0)))
    p = np.vstack(allpts)
    lo, hi = p.min(0), p.max(0)
    print(f'robot collision box: x [{lo[0]:+.3f}, {hi[0]:+.3f}]  y [{lo[1]:+.3f}, {hi[1]:+.3f}]  '
          f'z [{lo[2]:+.3f}, {hi[2]:+.3f}]  ({len(rows)} links with collisions, {len(q0)} initial values)')
    print('furthest in x:')
    for name, mn, mx in sorted(rows, key=lambda r: -max(abs(r[1][0]), abs(r[2][0])))[:a.top]:
        print(f'  {name:32s} x [{mn[0]:+.3f}, {mx[0]:+.3f}]  z [{mn[2]:+.3f}, {mx[2]:+.3f}]')


if __name__ == '__main__':
    main()
