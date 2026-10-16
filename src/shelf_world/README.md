# shelf_world

Gazebo Harmonic world: two shelves facing each other across a 1 m aisle, 13 boards each,
19 trays per board, 34 vials per tray (494 trays, 16 796 vials).

Frame: x across the aisle, y along the shelves, z up, origin on the floor at the aisle centre.
Shelf A at x = -0.81, shelf B at x = +0.81. Tray names: `tray_<shelf>_L<board 00-12>_T<slot 01-19>`,
board 00 is the bottom board (top at 0.36 m), board 12 the top (2.04 m).

## Run

```bash
colcon build --packages-select shelf_world && source install/setup.bash
ros2 launch shelf_world shelf_world.launch.py                                   # with GUI
ros2 launch shelf_world shelf_world.launch.py extra_gz_args:="-s --headless-rendering"  # server only
```

`/clock` is bridged to ROS 2.

## Change the world

Edit the constants in `scripts/generate_world.py`, then regenerate:

```bash
python3 scripts/generate_world.py                                   # all trays static
python3 scripts/generate_world.py --dynamic-level A:5               # board 5 of shelf A pickable
python3 scripts/generate_world.py --dynamic tray_A_L05_T10 tray_B_L00_T01
```

Each run writes `worlds/shelf_aisle.sdf` (Harmonic) and `worlds/shelf_aisle_fortress.sdf` (same world,
Fortress system names, for Fortress robots). Both load the IMU and contact systems for robot sensors.

Static trays have visuals only; each board's run of static trays shares two collision boxes in
the shelf model. With 3 boxes on every tray, DART ran at RTF 0.16; merged, it runs at 1.0.
Dynamic trays get their own collisions and a mass of 2.44 kg (assumed).

Assumed, not in the sketches: post depth 38 mm, tray body 25 mm tall with a 3 mm floor,
handle bar 20 x 20 mm at 45-65 mm, vial columns 29 mm apart, tray and vial masses.
