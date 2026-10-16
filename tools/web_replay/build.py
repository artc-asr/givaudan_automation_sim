#!/usr/bin/env python3
"""build.py: turn a recorded demo run into the web replay's data (docs/data/).

    ros2 launch givaudan_bringup demo.launch.py plan:=reachability/plans/task1_L08.json \\
        record:=/tmp/run_task1 gui:=false
    env -u PYTHONPATH deps/curobo-venv/bin/python tools/web_replay/build.py /tmp/run_task1   # -> docs/data/task1/

Adapted from tccc_hmlv_automation_sim's tools/web_replay/build.py. Reads what
givaudan_bringup's record_run.py wrote (recording.json, robot.urdf, world.sdf, plan.json)
and writes, for docs/index.html, into docs/data/<run id>/:

    run.json                 samples (joints, base, moving objects), steps, logs, facts
    scene.json               the world's boxes / cylinders, and mesh placements
    robot/robot.urdf         visuals only, mesh paths made relative
    robot/meshes/*.stl       the URDF's meshes, decimated to --face-budget faces in all

Mesh visuals (the shared tray-board and vial-block OBJs) go to docs/data/common/ once.
docs/data/index.json lists the demos. Needs trimesh and pymeshlab (both in the cuRobo venv).
"""
import argparse
import json
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / "docs" / "data"
LABELS = {"task1": "Task 1: pull + suction pick", "task2": "Task 2: whole tray, two hands"}
ROBOT_NAMES = {"rby1m": "Rainbow RB-Y1 (Model M)"}
MESH_FACES = 12000             # per robot mesh, at most
MIN_PART = 0.005               # drop loose robot-mesh pieces smaller than this (m)


# ------------------------------------------------------------------ robot model
def find_package(name, cache={}):
    if name not in cache:
        cache[name] = None
        for xml in REPO.glob("**/package.xml"):
            if any(p in xml.parts for p in ("install", "build", "log")):
                continue
            m = re.search(r"<name>\s*([^<\s]+)\s*</name>", xml.read_text())
            if m and m.group(1) == name:
                cache[name] = xml.parent
                break
    return cache[name]


def resolve_mesh(uri):
    if uri.startswith("package://"):
        pkg, _, rel = uri[len("package://"):].partition("/")
        root = find_package(pkg)
        return root / rel if root else None
    if uri.startswith("file://"):
        return Path(uri[len("file://"):])
    return Path(uri)


def drop_specks(mesh, size=MIN_PART):
    parts = mesh.split(only_watertight=False)
    keep = [p for p in parts if p.extents.max() >= size]
    if len(keep) == len(parts) or not keep:
        return mesh
    return trimesh.util.concatenate(keep)


def decimate(mesh, faces):
    import pymeshlab
    if len(mesh.faces) <= faces:
        return mesh
    ms = pymeshlab.MeshSet()
    ms.add_mesh(pymeshlab.Mesh(mesh.vertices, mesh.faces))
    ms.meshing_merge_close_vertices()
    ms.meshing_decimation_quadric_edge_collapse(
        targetfacenum=faces, preserveboundary=True, preservenormal=True,
        preservetopology=True, optimalplacement=True, planarquadric=True, qualitythr=0.3)
    m = ms.current_mesh()
    return trimesh.Trimesh(m.vertex_matrix(), m.face_matrix(), process=True)


def build_robot(urdf_text, out, budget):
    root = ET.fromstring(urdf_text)
    for tag in ("gazebo", "ros2_control", "transmission"):       # the page draws visuals only
        for el in root.findall(tag):
            root.remove(el)
    for link in root.findall("link"):
        for el in link.findall("collision") + link.findall("inertial"):
            link.remove(el)
    for el in root.iter("axis"):                                 # urdf-loader has no URDF defaults
        el.set("xyz", el.get("xyz") or "1 0 0")
    for el in root.iter("origin"):
        for k in ("xyz", "rpy"):
            el.set(k, el.get(k) or "0 0 0")
    meshes = {}
    for m in root.iter("mesh"):
        meshes.setdefault(m.get("filename"), []).append(m)
    loaded = {}
    for uri in meshes:
        path = resolve_mesh(uri)
        if path is None or not path.is_file():
            sys.exit(f"mesh not found: {uri}")
        loaded[uri] = trimesh.load(path, force="mesh")
    total = sum(len(m.faces) for m in loaded.values())
    ratio = min(1.0, budget / total)
    (out / "meshes").mkdir(parents=True, exist_ok=True)
    used, kept = set(), 0
    for uri, mesh in loaded.items():
        stem = name = Path(uri).stem
        k = 1
        while name.lower() in used:
            k += 1
            name = f"{stem}_{k}"
        used.add(name.lower())
        small = decimate(drop_specks(mesh), min(MESH_FACES, max(200, int(len(mesh.faces) * ratio))))
        kept += len(small.faces)
        small.export(out / "meshes" / f"{name}.stl")
        for el in meshes[uri]:
            scale = el.get("scale")
            el.set("filename", f"meshes/{name}.stl")
            if scale:
                el.set("scale", scale)
    (out / "robot.urdf").write_text(ET.tostring(root, encoding="unicode"))
    print(f"robot: {len(loaded)} meshes, {total} -> {kept} faces")
    return {j.get("name") for j in root.findall("joint") if j.get("type") != "fixed"}


# ------------------------------------------------------------------ scene
def _floats(text, n):
    vals = [float(v) for v in (text or "").split()]
    return (vals + [0.0] * n)[:n]


def _mat(pose_text):
    x, y, z, r, p, yw = _floats(pose_text, 6)
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yw), math.sin(yw))
    m = np.eye(4)
    m[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    m[:3, 3] = [x, y, z]
    return m


def _quat(r):
    q = trimesh.transformations.quaternion_from_matrix(r)      # w, x, y, z
    return [round(float(v), 5) for v in (q[1], q[2], q[3], q[0])]


def mesh_source(uri):
    """model://<model>/<path> -> the file in shelf_world/models."""
    if uri.startswith("model://"):
        return REPO / "src" / "shelf_world" / "models" / uri[len("model://"):]
    return Path(uri)


def mtl_colors(obj_path):
    mtl = obj_path.with_suffix(".mtl")
    colors, name = {}, None
    if mtl.exists():
        for line in mtl.read_text().splitlines():
            parts = line.split()
            if parts[:1] == ["newmtl"]:
                name = parts[1]
            elif parts[:1] == ["Kd"] and name:
                colors[name] = [float(v) for v in parts[1:4]] + [1.0]
    return colors


def build_scene(world_file, live):
    """Every visual of every model: primitives (boxes, cylinders) and mesh placements; `live`
    models in their own frame (the page poses them from the recording), the rest in the world."""
    world = ET.parse(world_file).getroot().find("world")
    prims, n_mesh = [], 0
    common = DATA / "common"
    common.mkdir(parents=True, exist_ok=True)
    for model in world.findall("model"):
        name = model.get("name", "")
        is_live = name in live
        mp = np.eye(4) if is_live else _mat(model.findtext("pose"))
        for link in model.findall("link"):
            lp = mp @ _mat(link.findtext("pose"))
            for vis in link.findall("visual"):
                g = vis.find("geometry")
                if g is None:
                    continue
                m = lp @ _mat(vis.findtext("pose"))
                dif = vis.findtext("material/diffuse")
                rgba = [round(c, 3) for c in _floats(dif, 4)] if dif else [0.6, 0.6, 0.6, 1.0]
                if g.find("box") is not None:
                    shape = {"type": "box", "size": _floats(g.findtext("box/size"), 3)}
                elif g.find("cylinder") is not None:
                    shape = {"type": "cylinder", "r": float(g.findtext("cylinder/radius")),
                             "h": float(g.findtext("cylinder/length"))}
                elif g.find("sphere") is not None:
                    shape = {"type": "sphere", "r": float(g.findtext("sphere/radius"))}
                elif g.find("mesh") is not None and not is_live:
                    src = mesh_source(g.findtext("mesh/uri"))
                    dst = common / src.name
                    if not dst.exists():
                        shutil.copy(src, dst)
                    colors = mtl_colors(src) or {}
                    colors.setdefault("", rgba)                        # OBJ without materials
                    shape = {"type": "mesh", "url": f"common/{src.name}",
                             "colors": {k: [round(c, 3) for c in v] for k, v in colors.items()}}
                    n_mesh += 1
                else:
                    continue                                    # the page draws its own floor
                prims.append({**shape, "model": name, "visual": vis.get("name", ""), "live": is_live,
                              "pos": [round(float(v), 4) for v in m[:3, 3]], "quat": _quat(m), "rgba": rgba})
    print(f"scene: {len(prims)} visuals ({n_mesh} mesh placements), {len(live)} moving models")
    return {"world": world.get("name"), "prims": prims}


# ------------------------------------------------------------------ run
def build_run(rec, movable, plan):
    names = rec["joint_names"]
    keep = [i for i, n in enumerate(names) if n in movable]
    joints = [[row[i] for i in keep] for row in rec["joints"]]
    objects, still = {}, {}
    for name, track in rec["objects"].items():
        a = np.array(track)
        moved = np.abs(a - a[0]).max() > 1e-3
        (objects if moved else still)[name] = track if moved else track[0]
    b = plan["base"]
    steps = [[s["phase"], s["label"], s["desc"]] for s in plan["steps"]]
    steps.append(["done", "Done", "Holding the final pose."])
    facts = [["Robot", ROBOT_NAMES.get(plan["robot"], plan["robot"])],
             ["Shelf level", f"{plan['level']} of 0-12"],
             ["Board top", f"{plan['board_top']:.2f} m"],
             ["Tray pulled", f"{plan['pull'] * 100:.0f} cm"],
             ["Base", f"x {b['x']:+.2f}, y {b['y']:+.2f} m"],
             ["Base yaw", f"{math.degrees(b['yaw']):+.0f}°"]]
    t = rec["t"]
    print(f"run: {len(t)} samples, {t[-1]:.0f} s, {len(keep)} joints, {len(objects)} moving objects, "
          f"{len(rec['phases'])} phase changes, {len(rec['logs'])} log lines")
    return {"id": rec.get("run_id", ""), "rate": rec["rate"], "t": t, "base_frame": rec["base_frame"],
            "joint_names": [names[i] for i in keep], "joints": joints, "base": rec["base"],
            "objects": objects, "objects_still": still, "phases": rec["phases"], "logs": rec["logs"],
            "phase_defs": steps, "facts": facts,
            # the camera: from the aisle end, slightly above, at the worked tray
            "view": {"target": [0.5, 0.0, round(plan["board_top"] + 0.02, 3)],
                     "pos": [0.25, -1.9, round(plan["board_top"] + 0.55, 3)]}}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--out", type=Path, help="default: docs/data/<run id>")
    ap.add_argument("--face-budget", type=int, default=120_000, help="robot mesh faces in all")
    a = ap.parse_args()

    rec = json.loads((a.run_dir / "recording.json").read_text())
    plan = json.loads((a.run_dir / "plan.json").read_text())
    a.out = a.out or DATA / (rec.get("run_id") or "run")
    if a.out.exists():
        shutil.rmtree(a.out)
    (a.out / "robot").mkdir(parents=True)
    movable = build_robot((a.run_dir / "robot.urdf").read_text(), a.out / "robot", a.face_budget)
    run = build_run(rec, movable, plan)
    scene = build_scene(a.run_dir / "world.sdf", set(rec["objects"]))
    (a.out / "run.json").write_text(json.dumps(run, separators=(",", ":")))
    (a.out / "scene.json").write_text(json.dumps(scene, separators=(",", ":")))
    size = sum(f.stat().st_size for f in a.out.rglob("*") if f.is_file())
    print(f"wrote {a.out} ({size / 1e6:.1f} MB)")
    write_index()


def write_index():
    demos = sorted(d.name for d in DATA.iterdir() if (d / "run.json").is_file())
    (DATA / "index.json").write_text(json.dumps(
        {"demos": [{"id": d, "label": LABELS.get(d, d)} for d in demos]}, indent=1))
    print(f"index: {', '.join(demos)}")


if __name__ == "__main__":
    main()
