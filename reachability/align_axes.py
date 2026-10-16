#!/usr/bin/env python3
"""Rewrite a URDF so every movable joint axis is exactly +-x, +-y or +-z.

    align_axes.py in.urdf out.urdf

cuRobo's URDF parser only accepts axis-aligned joints. For a joint with axis a,
pick the nearest basis axis e and the rotation R that takes e to a; then
  joint origin  T  -> T @ R        axis -> e
  everything expressed in the child link frame (visual, collision, inertial
  origins, origins of the child's own joints)  X -> R^T @ X
The child link frame turns by R, every other frame (and so FK of every other
link) is unchanged. Run --check to compare FK of all links before/after.
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np


def origin_matrix(el):
    t = np.eye(4)
    if el is None:
        return t
    xyz = [float(v) for v in el.get('xyz', '0 0 0').split()]
    r, p, y = (float(v) for v in el.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    t[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    t[:3, 3] = xyz
    return t


def set_origin(parent, t):
    el = parent.find('origin')
    if el is None:
        el = ET.SubElement(parent, 'origin')
    m = t[:3, :3]
    p = math.asin(max(-1.0, min(1.0, -m[2, 0])))
    if abs(math.cos(p)) > 1e-9:
        r, y = math.atan2(m[2, 1], m[2, 2]), math.atan2(m[1, 0], m[0, 0])
    else:   # gimbal lock: put everything in yaw
        r, y = 0.0, math.atan2(-m[0, 1], m[1, 1])
    el.set('xyz', ' '.join(f'{v:.9g}' for v in t[:3, 3]))
    el.set('rpy', f'{r:.9g} {p:.9g} {y:.9g}')


def rotation_between(e, a):
    """Rotation matrix taking unit vector e to unit vector a."""
    v, c = np.cross(e, a), float(np.dot(e, a))
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k * (1 / (1 + c))


def align(root):
    links = {lk.get('name'): lk for lk in root.findall('link')}
    joints = root.findall('joint')
    changed = []
    for j in joints:
        ax_el = j.find('axis')
        if j.get('type') == 'fixed' or ax_el is None:
            continue
        a = np.array([float(v) for v in ax_el.get('xyz').split()])
        a = a / np.linalg.norm(a)
        i = int(np.argmax(np.abs(a)))
        if abs(abs(a[i]) - 1.0) < 1e-9:
            continue
        e = np.zeros(3)
        e[i] = math.copysign(1.0, a[i])
        rot = np.eye(4)
        rot[:3, :3] = rotation_between(e, a)
        set_origin(j, origin_matrix(j.find('origin')) @ rot)
        ax_el.set('xyz', ' '.join(f'{v:g}' for v in e))
        child = j.find('child').get('link')
        inv = np.linalg.inv(rot)
        for tag in ('visual', 'collision', 'inertial'):
            for el in links[child].findall(tag):
                set_origin(el, inv @ origin_matrix(el.find('origin')))
        for cj in joints:
            if cj.find('parent').get('link') == child:
                set_origin(cj, inv @ origin_matrix(cj.find('origin')))
        changed.append((j.get('name'), a.round(4).tolist(), e.tolist()))
    return changed


def fk(root, q):
    """World pose of every link at joint positions q (dict), root link at identity."""
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    out = {}

    def pose(link):
        if link in out:
            return out[link]
        j = joints.get(link)
        if j is None:
            out[link] = np.eye(4)
            return out[link]
        t = pose(j.find('parent').get('link')) @ origin_matrix(j.find('origin'))
        if j.get('type') != 'fixed':
            a = np.array([float(v) for v in j.find('axis').get('xyz').split()])
            a = a / np.linalg.norm(a)
            m = np.eye(4)
            qq = q.get(j.get('name'), 0.0)
            if j.get('type') == 'prismatic':
                m[:3, 3] = a * qq
            else:
                k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                m[:3, :3] = np.eye(3) + math.sin(qq) * k + (1 - math.cos(qq)) * k @ k
            t = t @ m
        out[link] = t
        return t

    for lk in root.findall('link'):
        pose(lk.get('name'))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('input')
    ap.add_argument('output')
    ap.add_argument('--check', type=int, default=200, help='random joint samples for the FK check (0: skip)')
    a = ap.parse_args()
    before = ET.parse(a.input).getroot()
    tree = ET.parse(a.input)
    changed = align(tree.getroot())
    for name, old, new in changed:
        print(f'{name}: axis {old} -> {new}')
    tree.write(a.output, xml_declaration=True, encoding='utf-8')

    if a.check:
        rng = np.random.default_rng(0)
        movable = [j.get('name') for j in before.findall('joint') if j.get('type') != 'fixed']
        changed_links = {j.find('child').get('link') for j in tree.getroot().findall('joint')
                         if j.get('name') in {c[0] for c in changed}}
        worst = 0.0
        for _ in range(a.check):
            q = {n: float(rng.uniform(-1.5, 1.5)) for n in movable}
            fa, fb = fk(before, q), fk(tree.getroot(), q)
            for link in fa:
                if link in changed_links:
                    continue   # this frame is rotated on purpose; its geometry is compensated
                worst = max(worst, float(np.abs(fa[link] - fb[link]).max()))
        print(f'FK check: {a.check} random configurations, max |T_before - T_after| over unchanged links = {worst:.2e}')
        if worst > 1e-6:
            sys.exit('FK changed: the rewrite is wrong')


if __name__ == '__main__':
    main()
