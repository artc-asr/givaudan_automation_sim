#!/usr/bin/env python3
"""Generate the shelf-aisle Gazebo world (SDF 1.9) and the shared vial-block mesh.

Frame: x across the aisle, y along the shelves, z up, origin on the floor at the aisle
centre. Shelf A sits at x < 0, shelf B at x > 0, both with their front face on the aisle.
All dimensions in metres. Values marked "assumed" are not in the sketches.

Trays are static by default (cheap: the physics engine skips them). Make the ones a robot
will pick dynamic with --dynamic or --dynamic-level.
"""
import argparse
import math
from pathlib import Path

# ---- shelf (from the sketch) ----
SHELF_L = 1.22         # along y
SHELF_D = 0.62         # along x
AISLE = 1.00           # clear gap between the two front faces
POST = 0.038           # square section, assumed equal in both directions
POST_H = 2.20
BASE_Z = 0.33          # floor to underside of the bottom board
BASE_T = 0.03          # bottom board
GAP = 0.10             # clear height above each board
BOARD_T = 0.04
N_MODULES = 12         # gap + board, repeated above the bottom board

# ---- tray (from the sketch) ----
TRAY_W = 0.058
TRAY_L = 0.58          # 58 handle zone + 17 x 30 + 12 back
TRAY_H = 0.025         # assumed
TRAY_FLOOR = 0.003     # assumed, vials stand on it
HANDLE_ZONE = 0.058
BAR = 0.020            # handle bar section, assumed
BAR_Z = 0.045          # handle bar underside above tray bottom
UPRIGHT_T = 0.003      # side plates carrying the bar, assumed
VIAL_D = 0.028
VIAL_H = 0.080
PITCH = 0.030
COL_Y = 0.0145         # vial column offset from tray centre line
ROWS = 17
TRAYS_PER_BOARD = 19

# ---- demo (dynamic) tray and loose vial friction (assumed: plastic tray on a metal board) ----
TRAY_MU = 0.3
VIAL_MU = 0.1

# ---- dynamic tray mass (assumed) ----
TRAY_EMPTY_KG = 0.40
VIAL_KG = 0.06

PLUGIN_NS = {"harmonic": ("gz-sim", "gz::sim"), "fortress": ("ignition-gazebo", "ignition::gazebo")}

SHELVES = {"A": -1, "B": +1}   # side sign: shelf centre at x = side * SHELF_X

COLORS = {
    "post": (0.25, 0.28, 0.32),
    "board": (0.74, 0.77, 0.81),
    "tray": (0.33, 0.48, 0.68),
    "handle": (0.15, 0.17, 0.20),
    "vial": (0.85, 0.60, 0.20),
}


def board_list():
    """(z_bottom, thickness) for each board, bottom board first."""
    boards = [(BASE_Z, BASE_T)]
    top = BASE_Z + BASE_T
    for _ in range(N_MODULES):
        top += GAP + BOARD_T
        boards.append((top - BOARD_T, BOARD_T))
    return boards


def tray_slots_y():
    inner = SHELF_L - 2 * POST
    gap = (inner - TRAYS_PER_BOARD * TRAY_W) / (TRAYS_PER_BOARD + 1)
    return [-inner / 2 + gap * (i + 1) + TRAY_W * i + TRAY_W / 2 for i in range(TRAYS_PER_BOARD)]


def vial_positions():
    """Vial centres in the tray frame: origin at tray bottom centre, +x towards the handle."""
    out = []
    for r in range(ROWS):
        x = TRAY_L / 2 - HANDLE_ZONE - PITCH / 2 - r * PITCH
        for y in (-COL_Y, COL_Y):
            out.append((x, y))
    return out


# ---------------------------------------------------------------- SDF helpers

def f(*v):
    return " ".join(f"{x:.4f}".rstrip("0").rstrip(".") if x != 0 else "0" for x in v)


def material(name):
    r, g, b = COLORS[name]
    return (f"<material><ambient>{f(r, g, b)} 1</ambient><diffuse>{f(r, g, b)} 1</diffuse>"
            f"<specular>0.15 0.15 0.15 1</specular></material>")


def box(sx, sy, sz):
    return f"<box><size>{f(sx, sy, sz)}</size></box>"


def visual(name, pose, geom, mat):
    return f'<visual name="{name}"><pose>{f(*pose)} 0 0 0</pose><geometry>{geom}</geometry>{material(mat)}</visual>'


def collision(name, pose, geom, mu=None):
    # SDF 1.9 needs unique names among all children of a link, visuals and collisions alike.
    surface = (f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>"
               if mu is not None else "")
    return f'<collision name="{name}_col"><pose>{f(*pose)} 0 0 0</pose><geometry>{geom}</geometry>{surface}</collision>'


def tray_extents():
    """Tray-frame x ranges of the coarse collision boxes: (body), (vials + handle)."""
    vx = [x for x, _ in vial_positions()]
    upper = (min(vx) - VIAL_D / 2, TRAY_L / 2 - HANDLE_ZONE / 2 + BAR / 2)
    return (-TRAY_L / 2, TRAY_L / 2), upper


def static_tray_collisions(side, boards, dynamic_ids, shelf):
    """Merge each run of neighbouring static trays on a board into two boxes.

    DART slows to RTF 0.16 with 3 collision boxes on each of 494 static trays; with the
    merged boxes it runs in real time. Static trays then carry visuals only.
    """
    (bx0, bx1), (ux0, ux1) = tray_extents()
    flip = -side   # tray +x points at the aisle; in the shelf frame that is x * -side
    ys = tray_slots_y()
    parts = []
    for lvl, (z0, t) in enumerate(boards):
        top = z0 + t
        static = [i for i in range(TRAYS_PER_BOARD)
                  if f"tray_{shelf}_L{lvl:02d}_T{i + 1:02d}" not in dynamic_ids]
        runs, run = [], []
        for i in static:
            if run and i != run[-1] + 1:
                runs.append(run)
                run = []
            run.append(i)
        if run:
            runs.append(run)
        for k, r in enumerate(runs):
            y0, y1 = ys[r[0]] - TRAY_W / 2, ys[r[-1]] + TRAY_W / 2
            ym, ly = (y0 + y1) / 2, y1 - y0
            hz = TRAY_FLOOR + VIAL_H - TRAY_H
            parts.append(collision(f"trays_L{lvl:02d}_r{k}_body", (flip * (bx0 + bx1) / 2, ym, top + TRAY_H / 2),
                                   box(bx1 - bx0, ly, TRAY_H)))
            parts.append(collision(f"trays_L{lvl:02d}_r{k}_top", (flip * (ux0 + ux1) / 2, ym, top + TRAY_H + hz / 2),
                                   box(ux1 - ux0, ly, hz)))
    return parts


def shelf_model(name, side, extra=(), mesh_boards=()):
    cx = side * (AISLE / 2 + SHELF_D / 2)
    parts = list(extra)
    yaw = 0.0 if side < 0 else math.pi   # board mesh +x (handles) points at the aisle
    for lvl in mesh_boards:
        z0, t = board_list()[lvl]
        parts.append(f'<visual name="trays_L{lvl:02d}"><pose>{f(0, 0, z0 + t)} 0 0 {f(yaw)}</pose>'
                     '<geometry><mesh><uri>model://tray_board/meshes/tray_board.obj</uri></mesh></geometry></visual>')
    for i, (sx, sy) in enumerate([(-1, -1), (-1, 1), (1, -1), (1, 1)]):
        p = (sx * (SHELF_D / 2 - POST / 2), sy * (SHELF_L / 2 - POST / 2), POST_H / 2)
        g = box(POST, POST, POST_H)
        parts += [visual(f"post_{i}", p, g, "post"), collision(f"post_{i}", p, g)]
    for i, (z0, t) in enumerate(board_list()):
        p = (0, 0, z0 + t / 2)
        g = box(SHELF_D, SHELF_L - 2 * POST, t)
        parts += [visual(f"board_{i:02d}", p, g, "board"), collision(f"board_{i:02d}", p, g)]
    body = "\n      ".join(parts)
    return f"""  <model name="{name}">
    <static>true</static>
    <pose>{f(cx, 0, 0)} 0 0 0</pose>
    <link name="frame">
      {body}
    </link>
  </model>"""


def detachable(parent_link, child_model, child_link, topic, gz):
    """DetachableJoint from this model's link to a robot link (sim-only grasp / suction).
    Harmonic attaches as soon as the child model exists: the demo detaches at its start
    and re-attaches on <topic>/attach (fixed joint at the current relative pose)."""
    lib, ns = PLUGIN_NS[gz]
    return (f'<plugin filename="{lib}-detachable-joint-system" name="{ns}::systems::DetachableJoint">'
            f'<parent_link>{parent_link}</parent_link><child_model>{child_model}</child_model>'
            f'<child_link>{child_link}</child_link><attach_topic>{topic}/attach</attach_topic>'
            f'<detach_topic>{topic}/detach</detach_topic><output_topic>{topic}/state</output_topic>'
            f'<suppress_child_warning>true</suppress_child_warning></plugin>')


def pose_publisher(gz):
    """Publishes the model's pose on /model/<name>/pose (for the recorder)."""
    lib, ns = PLUGIN_NS[gz]
    return (f'<plugin filename="{lib}-pose-publisher-system" name="{ns}::systems::PosePublisher">'
            '<publish_link_pose>false</publish_link_pose><publish_model_pose>true</publish_model_pose>'
            '<use_pose_vector_msg>false</use_pose_vector_msg><update_frequency>50</update_frequency></plugin>')


def vial_model(name, pose_xyz, grip, gz):
    """One loose vial (the one a suction tool picks), standing on its tray's floor."""
    m, r, h = VIAL_KG, VIAL_D / 2, VIAL_H
    ixx, izz = m * (3 * r * r + h * h) / 12, m * r * r / 2
    geom = f"<cylinder><radius>{f(r)}</radius><length>{f(h)}</length></cylinder>"
    # 1 mm slimmer collision and low friction: it stands 1 mm from its neighbours and would
    # otherwise jam when lifted straight out
    col_geom = f"<cylinder><radius>{f(r - 0.001)}</radius><length>{f(h)}</length></cylinder>"
    plugins = [pose_publisher(gz)] + ([detachable("vial", grip[0], grip[1], f"/givaudan/{name}/{grip[1]}", gz)]
                                      if grip else [])
    return f"""  <model name="{name}">
    <pose>{f(*pose_xyz)} 0 0 0</pose>
    <link name="vial">
      <inertial><pose>0 0 {f(h / 2)} 0 0 0</pose><mass>{f(m)}</mass><inertia><ixx>{ixx:.6g}</ixx><iyy>{ixx:.6g}</iyy>
        <izz>{izz:.6g}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      {visual("body", (0, 0, h / 2), geom, "vial")}
      {collision("body", (0, 0, h / 2), col_geom, mu=VIAL_MU)}
    </link>
    {"".join(plugins)}
  </model>"""


def tray_model(name, pose_xyz, yaw, dynamic, grips=(), pick=None, gz="harmonic"):
    """`grips`: [(model, link)] welds for the robot's hands; `pick`: (row, col) of the vial
    taken out as its own model (then the tray is hollow, with one collision per vial)."""
    vx = [x for x, _ in vial_positions()]
    vial_x0, vial_x1 = min(vx) - VIAL_D / 2, max(vx) + VIAL_D / 2
    bar_x = TRAY_L / 2 - HANDLE_ZONE / 2
    # Static trays get their collisions merged into the shelf (see static_tray_collisions).
    up_h = BAR_Z - TRAY_H
    vis = [
        visual("body", (0, 0, TRAY_H / 2), box(TRAY_L, TRAY_W, TRAY_H), "tray"),
        visual("bar", (bar_x, 0, BAR_Z + BAR / 2), box(BAR, TRAY_W, BAR), "handle"),
        visual("upright_l", (bar_x, TRAY_W / 2 - UPRIGHT_T / 2, TRAY_H + up_h / 2), box(BAR, UPRIGHT_T, up_h), "handle"),
        visual("upright_r", (bar_x, -TRAY_W / 2 + UPRIGHT_T / 2, TRAY_H + up_h / 2), box(BAR, UPRIGHT_T, up_h), "handle"),
    ]
    vials = vial_positions()
    picked = vial_xy(*pick) if pick else None
    cyl = f"<cylinder><radius>{f(VIAL_D / 2)}</radius><length>{f(VIAL_H)}</length></cylinder>"
    if dynamic:   # one visual per vial: a picked vial leaves a gap, and the replay page needs primitives
        vis += [visual(f"vial_{i:02d}", (x, y, TRAY_FLOOR + VIAL_H / 2), cyl, "vial")
                for i, (x, y) in enumerate(vials) if (x, y) != picked]
    else:
        vis.append(f'<visual name="vials"><geometry><mesh><uri>model://vial_block/meshes/vial_block.obj</uri></mesh></geometry>{material("vial")}</visual>')
    # Coarse collision: body, handle, and one box around all 34 vials.
    col = [
        collision("body", (0, 0, TRAY_H / 2), box(TRAY_L, TRAY_W, TRAY_H)),
        collision("bar", (bar_x, 0, BAR_Z + BAR / 2), box(BAR, TRAY_W, BAR)),
        collision("vials", ((vial_x0 + vial_x1) / 2, 0, TRAY_H + (TRAY_FLOOR + VIAL_H - TRAY_H) / 2),
                  box(vial_x1 - vial_x0, TRAY_W, TRAY_FLOOR + VIAL_H - TRAY_H)),
    ]
    if pick:      # hollow tray (floor + walls) and one collision per vial: the loose vial stands on the floor
        wall = 0.003
        col = [
            collision("floor", (0, 0, TRAY_FLOOR / 2), box(TRAY_L, TRAY_W, TRAY_FLOOR), mu=TRAY_MU),
            collision("side_l", (0, TRAY_W / 2 - wall / 2, TRAY_H / 2), box(TRAY_L, wall, TRAY_H)),
            collision("side_r", (0, -TRAY_W / 2 + wall / 2, TRAY_H / 2), box(TRAY_L, wall, TRAY_H)),
            collision("end_back", (-TRAY_L / 2 + wall / 2, 0, TRAY_H / 2), box(wall, TRAY_W, TRAY_H)),
            collision("end_front", (TRAY_L / 2 - wall / 2, 0, TRAY_H / 2), box(wall, TRAY_W, TRAY_H)),
            collision("bar", (bar_x, 0, BAR_Z + BAR / 2), box(BAR, TRAY_W, BAR)),
        ] + [collision(f"vial_{i:02d}", (x, y, TRAY_FLOOR + VIAL_H / 2), cyl)
             for i, (x, y) in enumerate(vials) if (x, y) != picked]
    inertial = ""
    if dynamic:
        m = TRAY_EMPTY_KG + ROWS * 2 * VIAL_KG
        h = TRAY_FLOOR + VIAL_H
        ixx = m / 12 * (TRAY_W ** 2 + h ** 2)
        iyy = m / 12 * (TRAY_L ** 2 + h ** 2)
        izz = m / 12 * (TRAY_L ** 2 + TRAY_W ** 2)
        inertial = (f"<inertial><pose>0 0 {f(h / 2)} 0 0 0</pose><mass>{f(m)}</mass><inertia>"
                    f"<ixx>{ixx:.6g}</ixx><iyy>{iyy:.6g}</iyy><izz>{izz:.6g}</izz>"
                    f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")
    body = "\n      ".join(([inertial] if inertial else []) + vis + (col if dynamic else []))
    plugins = ([pose_publisher(gz)] if dynamic else []) + [
        detachable("tray", m, lk, f"/givaudan/{name}/{lk}", gz) for m, lk in grips]
    return f"""  <model name="{name}">
    <static>{'false' if dynamic else 'true'}</static>
    <pose>{f(*pose_xyz)} 0 0 {f(yaw)}</pose>
    <link name="tray">
      {body}
    </link>
    {"".join(plugins)}
  </model>"""


def vial_xy(row, col):
    """Tray-frame centre of vial `row` (1 = next to the handle) in column `col` (1: +y, 2: -y)."""
    xs = sorted({x for x, _ in vial_positions()}, reverse=True)
    return xs[row - 1], COL_Y if col == 1 else -COL_Y


def world_sdf(dynamic_ids, gz="harmonic", grips=None, picks=None):
    """`grips`: {tray id: [(model, link)]}; `picks`: {tray id: (row, col, (model, link) or None)}."""
    grips, picks = grips or {}, picks or {}
    dynamic_ids = set(dynamic_ids) | set(grips) | set(picks)
    boards = board_list()
    mixed = {(s, lvl) for s in SHELVES for lvl in range(len(boards))
             if any(f"tray_{s}_L{lvl:02d}_T{t:02d}" in dynamic_ids for t in range(1, TRAYS_PER_BOARD + 1))}
    models = [shelf_model(f"shelf_{s}", side, static_tray_collisions(side, boards, dynamic_ids, s),
                          [lvl for lvl in range(len(boards)) if (s, lvl) not in mixed])
              for s, side in SHELVES.items()]
    n_dyn = 0
    for s, side in SHELVES.items():
        cx = side * (AISLE / 2 + SHELF_D / 2)   # tray is centred on the shelf depth
        yaw = 0.0 if side < 0 else math.pi      # tray +x (handle) points at the aisle
        for lvl, (z0, t) in enumerate(boards):
            if (s, lvl) not in mixed:
                continue   # drawn by the shelf's board mesh
            for slot, y in enumerate(tray_slots_y(), start=1):
                tid = f"tray_{s}_L{lvl:02d}_T{slot:02d}"
                dyn = tid in dynamic_ids
                n_dyn += dyn
                # dynamic trays start 1 mm above the board so they settle instead of interpenetrating
                pose = (cx, y, z0 + t + (0.001 if dyn else 0))
                pick = picks.get(tid)
                models.append(tray_model(tid, pose, yaw, dyn, grips.get(tid, ()), pick[:2] if pick else None, gz))
                if pick:
                    vx_t, vy_t = vial_xy(*pick[:2])
                    c, sn = math.cos(yaw), math.sin(yaw)
                    models.append(vial_model(f"vial_{tid[5:]}_r{pick[0]:02d}c{pick[1]}",
                                             (pose[0] + c * vx_t - sn * vy_t, pose[1] + sn * vx_t + c * vy_t,
                                              pose[2] + TRAY_FLOOR + 0.0005), pick[2], gz))
    unknown = dynamic_ids - {m.split('"')[1] for m in models}   # every dynamic tray's board is in mixed
    if unknown:
        raise SystemExit(f"unknown tray ids: {sorted(unknown)}")
    body = "\n".join(models)
    plugins = world_plugins(gz)
    sdf = f"""<?xml version="1.0"?>
<!-- Generated by scripts/generate_world.py. Edit the script, not this file. -->
<sdf version="1.9">
<world name="shelf_aisle">
  <physics name="1ms" type="ignored">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
{plugins}

  <scene>
    <ambient>0.6 0.6 0.6 1</ambient>
    <background>0.8 0.84 0.9 1</background>
    <grid>false</grid>
  </scene>

  <light type="directional" name="sun">
    <cast_shadows>true</cast_shadows>
    <pose>0 0 10 0 0 0</pose>
    <diffuse>0.8 0.8 0.8 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <direction>-0.4 0.2 -0.9</direction>
  </light>

  <model name="ground_plane">
    <static>true</static>
    <link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
      <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
    </link>
  </model>

{body}
</world>
</sdf>
"""
    return sdf, n_dyn


def world_plugins(gz):
    """World systems. IMU and contact are there for the robots' sensors (rochestra's worlds load them too)."""
    lib, ns = PLUGIN_NS[gz]
    systems = [("physics", "Physics"), ("user-commands", "UserCommands"), ("scene-broadcaster", "SceneBroadcaster"),
               ("imu", "Imu"), ("contact", "Contact")]
    lines = [f'  <plugin filename="{lib}-{f}-system" name="{ns}::systems::{n}"/>' for f, n in systems]
    lines.append(f'  <plugin filename="{lib}-sensors-system" name="{ns}::systems::Sensors">\n'
                 '    <render_engine>ogre2</render_engine>\n  </plugin>')
    return "\n".join(lines)


def vial_block_obj(segments=16):
    """34 closed cylinders in the tray frame, one mesh shared by every tray."""
    v, vn, faces = [], [], []
    r = VIAL_D / 2
    z0, z1 = TRAY_FLOOR, TRAY_FLOOR + VIAL_H
    vn.append((0, 0, -1))
    vn.append((0, 0, 1))
    for k in range(segments):
        a = 2 * math.pi * k / segments
        vn.append((math.cos(a), math.sin(a), 0))
    for cx, cy in vial_positions():
        base = len(v)
        for k in range(segments):
            a = 2 * math.pi * k / segments
            x, y = cx + r * math.cos(a), cy + r * math.sin(a)
            v.append((x, y, z0))
            v.append((x, y, z1))
        v.append((cx, cy, z0))
        v.append((cx, cy, z1))
        cb, ct = base + 2 * segments, base + 2 * segments + 1   # 0-based cap centres
        for k in range(segments):
            k2 = (k + 1) % segments
            b0, t0, b1, t1 = base + 2 * k, base + 2 * k + 1, base + 2 * k2, base + 2 * k2 + 1
            n0, n1 = 3 + k, 3 + k2                               # 1-based normal indices
            faces.append(((b0, n0), (b1, n1), (t1, n1)))
            faces.append(((b0, n0), (t1, n1), (t0, n0)))
            faces.append(((cb, 1), (b1, 1), (b0, 1)))
            faces.append(((ct, 2), (t0, 2), (t1, 2)))
    lines = ["# vial block: 34 vials, tray frame (origin tray bottom centre, +x to handle), metres"]
    lines += [f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in v]
    lines += [f"vn {x:.5f} {y:.5f} {z:.5f}" for x, y, z in vn]
    lines += ["f " + " ".join(f"{i + 1}//{n}" for i, n in tri) for tri in faces]
    return "\n".join(lines) + "\n"


class ObjMesh:
    """Triangle mesh written as OBJ + MTL, faces grouped by material."""

    def __init__(self):
        self.v, self.vn, self.faces = [], [], {}

    def _vn(self, n):
        self.vn.append(n)
        return len(self.vn)   # 1-based

    def box(self, mat, centre, size):
        cx, cy, cz = centre
        hx, hy, hz = (d / 2 for d in size)
        base = len(self.v)
        for dx in (-1, 1):
            for dy in (-1, 1):
                for dz in (-1, 1):
                    self.v.append((cx + dx * hx, cy + dy * hy, cz + dz * hz))
        idx = lambda dx, dy, dz: base + (dx > 0) * 4 + (dy > 0) * 2 + (dz > 0)  # noqa: E731
        fl = self.faces.setdefault(mat, [])
        for axis in range(3):
            for sgn in (-1, 1):
                n = [0, 0, 0]
                n[axis] = sgn
                ni = self._vn(tuple(n))
                # the four corners of this face, counter-clockwise seen from outside
                a, b = [i for i in range(3) if i != axis]
                quad = []
                for ua, ub in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    c = [0, 0, 0]
                    c[axis], c[a], c[b] = sgn, ua, ub
                    quad.append(idx(*c))
                if (sgn > 0) != ((axis + 1) % 3 == a):   # flip winding so the normal points out
                    quad.reverse()
                fl.append(((quad[0], ni), (quad[1], ni), (quad[2], ni)))
                fl.append(((quad[0], ni), (quad[2], ni), (quad[3], ni)))

    def cylinder(self, mat, cx, cy, z0, z1, r, segments=16):
        fl = self.faces.setdefault(mat, [])
        bottom, top = self._vn((0, 0, -1)), self._vn((0, 0, 1))
        normals = [self._vn((math.cos(2 * math.pi * k / segments), math.sin(2 * math.pi * k / segments), 0))
                   for k in range(segments)]
        base = len(self.v)
        for k in range(segments):
            a = 2 * math.pi * k / segments
            x, y = cx + r * math.cos(a), cy + r * math.sin(a)
            self.v += [(x, y, z0), (x, y, z1)]
        self.v += [(cx, cy, z0), (cx, cy, z1)]
        cb, ct = base + 2 * segments, base + 2 * segments + 1
        for k in range(segments):
            k2 = (k + 1) % segments
            b0, t0, b1, t1 = base + 2 * k, base + 2 * k + 1, base + 2 * k2, base + 2 * k2 + 1
            n0, n1 = normals[k], normals[k2]
            fl += [((b0, n0), (b1, n1), (t1, n1)), ((b0, n0), (t1, n1), (t0, n0)),
                   ((cb, bottom), (b1, bottom), (b0, bottom)), ((ct, top), (t0, top), (t1, top))]

    def write(self, obj_path, header):
        mtl = obj_path.with_suffix(".mtl")
        lines = [f"# {header}", f"mtllib {mtl.name}"]
        lines += [f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in self.v]
        lines += [f"vn {x:.4f} {y:.4f} {z:.4f}" for x, y, z in self.vn]
        for mat, faces in self.faces.items():
            lines.append(f"usemtl {mat}")
            lines += ["f " + " ".join(f"{i + 1}//{n}" for i, n in tri) for tri in faces]
        obj_path.write_text("\n".join(lines) + "\n")
        m = []
        for mat in self.faces:
            r, g, b = COLORS[mat]
            m += [f"newmtl {mat}", f"Ka {r} {g} {b}", f"Kd {r} {g} {b}", "Ks 0.15 0.15 0.15", "d 1", ""]
        mtl.write_text("\n".join(m))


def tray_board_mesh():
    """All 19 trays of one board with their vials, in the board frame: origin on the board top
    at the shelf centre, +x towards the aisle (the tray handle side). One mesh for every
    all-static board: a camera or lidar then draws 3 sub-meshes per board instead of 95 visuals."""
    mesh = ObjMesh()
    bar_x = TRAY_L / 2 - HANDLE_ZONE / 2
    up_h = BAR_Z - TRAY_H
    for y in tray_slots_y():
        mesh.box("tray", (0, y, TRAY_H / 2), (TRAY_L, TRAY_W, TRAY_H))
        mesh.box("handle", (bar_x, y, BAR_Z + BAR / 2), (BAR, TRAY_W, BAR))
        for sy in (1, -1):
            mesh.box("handle", (bar_x, y + sy * (TRAY_W / 2 - UPRIGHT_T / 2), TRAY_H + up_h / 2),
                     (BAR, UPRIGHT_T, up_h))
        for vx, vy in vial_positions():
            mesh.cylinder("vial", vx, y + vy, TRAY_FLOOR, TRAY_FLOOR + VIAL_H, VIAL_D / 2)
    return mesh


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>vial_block</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Mesh of the 34 vials in one tray, referenced by every tray in shelf_aisle.sdf.</description>
</model>
"""

MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="vial_block">
    <static>true</static>
    <link name="link">
      <visual name="visual"><geometry><mesh><uri>meshes/vial_block.obj</uri></mesh></geometry></visual>
    </link>
  </model>
</sdf>
"""


def main():
    pkg = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dynamic", nargs="*", default=[], metavar="TRAY_ID",
                    help="tray ids to make dynamic, e.g. tray_A_L05_T10")
    ap.add_argument("--dynamic-level", nargs="*", default=[], metavar="SHELF:LEVEL",
                    help="make a whole board of trays dynamic, e.g. A:5")
    ap.add_argument("--grip", nargs="*", default=[], metavar="TRAY=MODEL:LINK[,LINK]",
                    help="sim-only welds from a tray to robot links, e.g. tray_B_L08_T10=rby1:ee_left,ee_right")
    ap.add_argument("--pick-vial", nargs="*", default=[], metavar="TRAY:ROW:COL[=MODEL:LINK]",
                    help="make one vial its own model (row 1 next to the handle, col 1/2), optionally with "
                         "a suction weld, e.g. tray_B_L08_T10:3:1=rby1:ee_right")
    ap.add_argument("-o", "--output", type=Path, default=pkg / "worlds" / "shelf_aisle.sdf",
                    help="the world file (Gazebo Harmonic)")
    args = ap.parse_args()

    dyn = set(args.dynamic)
    for spec in args.dynamic_level:
        s, lvl = spec.split(":")
        dyn |= {f"tray_{s}_L{int(lvl):02d}_T{t:02d}" for t in range(1, TRAYS_PER_BOARD + 1)}

    grips = {}
    for spec in args.grip:
        tray, rest = spec.split("=")
        model, links = rest.split(":")
        grips[tray] = [(model, lk) for lk in links.split(",")]
    picks = {}
    for spec in args.pick_vial:
        where, _, who = spec.partition("=")
        tray, row, col = where.split(":")
        picks[tray] = (int(row), int(col), tuple(who.split(":")) if who else None)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    sdf, n_dyn = world_sdf(dyn, "harmonic", grips, picks)
    args.output.write_text(sdf)

    mdir = pkg / "models" / "vial_block"
    (mdir / "meshes").mkdir(parents=True, exist_ok=True)
    (mdir / "meshes" / "vial_block.obj").write_text(vial_block_obj())
    (mdir / "model.config").write_text(MODEL_CONFIG)
    (mdir / "model.sdf").write_text(MODEL_SDF)

    bdir = pkg / "models" / "tray_board"
    (bdir / "meshes").mkdir(parents=True, exist_ok=True)
    tray_board_mesh().write(bdir / "meshes" / "tray_board.obj",
                            "one board of 19 trays + vials, board frame (origin board top centre, +x to handles), metres")
    (bdir / "model.config").write_text(MODEL_CONFIG.replace("vial_block", "tray_board").replace(
        "Mesh of the 34 vials in one tray, referenced by every tray", "19 trays with vials, one board, referenced by every all-static board"))
    (bdir / "model.sdf").write_text(MODEL_SDF.replace("vial_block", "tray_board").replace(
        "meshes/vial_block.obj", "meshes/tray_board.obj"))

    n_trays = len(SHELVES) * len(board_list()) * TRAYS_PER_BOARD
    print(f"wrote {args.output}: 2 shelves, {n_trays} trays ({n_dyn} dynamic), "
          f"{n_trays * ROWS * 2} vials")


if __name__ == "__main__":
    main()
