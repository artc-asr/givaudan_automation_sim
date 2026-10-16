# reachability

Which shelf level can each robot work? Solved with [cuRobo](https://github.com/NVlabs/curobo)
(GPU, collision-aware, both hands at once). Plain Python, no ROS: results are JSON
under `results/`, for the GitHub Pages report later.

## Setup (once)

```bash
git clone https://github.com/NVlabs/curobo.git deps/curobo        # tested at 78fd485 (2026-09-10)
python3 -m venv deps/curobo-venv
deps/curobo-venv/bin/pip install "./deps/curobo[cu13-torch]"      # driver >= 580.65; ~8 GB
```

Run cuRobo with ROS's `PYTHONPATH` removed: `env -u PYTHONPATH deps/curobo-venv/bin/python ...`

## Robot model (per robot)

1. URDF from rochestra (xacro), e.g. `robots/rby1m.urdf`.
2. `python3 align_axes.py robots/rby1m.urdf robots/rby1m_curobo.urdf`: cuRobo only takes
   axis-aligned joints (rby1's shoulders are tilted 20 deg). The script turns those joint frames
   and checks FK of all other links is unchanged (max 1e-9 m over 200 random poses).
3. `build_robot.py ... -o robots/rby1m.yml`: collision spheres fitted to the meshes (MorphIt)
   + self-collision ignore matrix, ~2 min.

## Sweep (`sweep.py`)

```bash
env -u PYTHONPATH ../deps/curobo-venv/bin/python sweep.py --robot rby1m --task 1|2 --base fixed|mobile
env -u PYTHONPATH ../deps/curobo-venv/bin/python render.py results/rby1m_task2_mobile.json 5   # picture
python3 build_docs.py      # GitHub Pages report: ../docs/index.html (from results/ and figures/)
```

- **Task 1**: left hand on the handle bar, tray pulled 25 cm (back end still on the board); right
  hand = emulated suction tool (13 cm below `ee_right`, pointing down) on a vial in a row clear of
  the board above (rows 1-4 of 17).
- **Task 2**: tray fully out (back end 10 cm in front of the shelf face, lifted 5 mm); left hand on
  the bar (from the front or from above), right hand on the back end from above, fingers either
  side of the tray (80 mm open).
- **Base**: `fixed` = aisle centre facing shelf B (the Gazebo spawn); `mobile` = x, y, yaw free in
  the aisle (cuRobo virtual base joints).
- **Scene**: the Gazebo world's collision boxes (from `shelf_world`'s generator) minus the target
  tray, a keep-out box under each shelf (assumed: no driving under the bottom board; rby1's base is
  0.38 m tall, the gap 0.33 m), and in task 2 the held tray with 10 cm cut out around each hand.
- **Post-check** (exact): every collision sphere except the hands' must clear the *whole* tray;
  the 16 best IK solutions per problem are tried in order.
- Seeds: 256 for task 1, 1024 for task 2 (its solutions are narrow: 256 found none).

### rby1 (Model M), shelf levels that work (board top 0.36 m + 0.14 m per level)

| | task 1 (pull + suction) | task 2 (full tray, two hands) |
|---|---|---|
| fixed base | 0-8 | 2-8 (torso twisted ~100 deg, leaning) |
| mobile base | 0-8 | 0-8 (stands beside the tray) |

Highest: **level 8, board top 1.48 m** in every case. Level 9 (1.62 m) misses by >= 16 mm
(task 1) and >= 67 mm (task 2). Independent FK on the original URDF: worst 2.7 mm.

Not checked: joint torques / payload (tray assumed 2.44 kg), balance, motion between poses
(only the key pose), real shelf details under the bottom board.
