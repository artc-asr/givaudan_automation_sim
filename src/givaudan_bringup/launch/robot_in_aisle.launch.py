"""One rochestra robot in the shelf aisle, between the two shelves, facing one of them.

    ros2 launch givaudan_bringup robot_in_aisle.launch.py robot:=rby1 face:=B
    ros2 launch givaudan_bringup robot_in_aisle.launch.py robot:=vb2 face:=A gui:=false

Robots: vb2 (Lanxin VB2), g1 (Galbot G1), rby1 (Rainbow RB-Y1 Model M, mecanum),
moz1 (SpiritAI Moz1), all on Gazebo Harmonic (rochestra D19): source scripts/env.sh first.

World frame (shelf_world README): x across the aisle, shelf A at x < 0, shelf B
at x > 0, aisle 1.0 m wide. face:=B is yaw 0 (robot +x towards shelf B),
face:=A is yaw pi. The robot is placed at (x, y), default the aisle centre.

Only calls each robot's rochestra bringup (roc_<robot> bringup.launch.py mode:=gz)
with this world and pose; nothing robot-specific is copied here.
"""
import math
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# robot: (rochestra core package, Gazebo version, GUI argument name, extra bringup args)
ROBOTS = {
    'vb2': ('roc_lanxin_vb2', 'harmonic', 'gz_gui', {}),
    'g1': ('roc_galbot_g1', 'harmonic', 'gz_gui', {}),
    'rby1': ('roc_rby1', 'harmonic', 'gui', {'model': 'm'}),   # Model M: mecanum base
    'moz1': ('roc_spiritai_moz1', 'harmonic', 'gui', {}),
}
WORLDS = {'harmonic': 'shelf_aisle.sdf'}
FACE_YAW = {'A': math.pi, 'B': 0.0}
# Physics as in each robot's own rochestra world (the robots were tuned there):
# vb2 empty_table 2 ms, g1 galbot_table 2 ms + Bullet, rby1 rby1_world 1 ms,
# moz1 moz1_pickplace 4 ms. (step [s], DART collision detector or None = FCL)
PHYSICS = {'vb2': (0.002, None), 'g1': (0.002, 'bullet'), 'rby1': (0.001, None), 'moz1': (0.004, None)}


def world_with_physics(world, robot):
    """Copy of the world file with this robot's <physics> block, in a temp dir."""
    step, detector = PHYSICS[robot]
    dart = f'<dart><collision_detector>{detector}</collision_detector></dart>' if detector else ''
    physics = (f'<physics name="{robot}" type="ignored"><max_step_size>{step}</max_step_size>'
               f'<real_time_factor>1.0</real_time_factor>{dart}</physics>')
    with open(world) as f:
        sdf, n = re.subn(r'<physics\b.*?</physics>', physics, f.read(), count=1, flags=re.S)
    if n != 1:
        raise RuntimeError(f'no <physics> block in {world}')
    out = os.path.join(tempfile.mkdtemp(prefix='givaudan_world_'), os.path.basename(world))
    with open(out, 'w') as f:
        f.write(sdf)
    return out


def launch_setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    robot, face = arg('robot'), arg('face').upper()
    if robot not in ROBOTS:
        raise ValueError(f'robot must be one of {sorted(ROBOTS)}, got {robot!r}')
    if face not in FACE_YAW:
        raise ValueError(f"face must be 'A' or 'B', got {face!r}")
    package, gz, gui_arg, extra = ROBOTS[robot]
    x, y = float(arg('x')), float(arg('y'))
    yaw = float(arg('yaw')) if arg('yaw') else FACE_YAW[face]

    world_share = get_package_share_directory('shelf_world')
    world = world_with_physics(arg('world') or os.path.join(world_share, 'worlds', WORLDS[gz]), robot)
    args = {'mode': 'gz', 'world': world, gui_arg: arg('gui'), **extra}
    if robot == 'moz1':
        # rochestra's moz1 launch has no pose arguments: it always spawns at the
        # world origin with yaw 0, i.e. the aisle centre facing shelf B.
        if (x, y, yaw) != (0.0, 0.0, 0.0):
            raise ValueError('moz1: rochestra spawns it at x=0 y=0 facing shelf B only '
                             '(its sim.launch.py has no x/y/yaw arguments yet)')
    else:
        args.update(x=str(x), y=str(y), yaw=str(yaw))

    models = os.path.join(world_share, 'models')   # model://vial_block
    return [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models),
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package), 'bringup', 'launch', 'bringup.launch.py')),
            launch_arguments=args.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', description='vb2 | g1 | rby1 | moz1'),
        DeclareLaunchArgument('face', default_value='B', description='shelf the robot faces: A (x < 0) or B (x > 0)'),
        DeclareLaunchArgument('x', default_value='0.0', description='across the aisle, 0 = centre'),
        DeclareLaunchArgument('y', default_value='0.0', description='along the shelves, 0 = centre'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo window'),
        DeclareLaunchArgument('yaw', default_value='', description='base yaw [rad]; overrides face'),
        DeclareLaunchArgument('world', default_value='',
                              description='world file (default: shelf_world for the robot\'s Gazebo version)'),
        OpaqueFunction(function=launch_setup),
    ])
