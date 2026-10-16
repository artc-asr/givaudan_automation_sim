"""Run (and optionally record) a planned shelf demo in Gazebo.

    ros2 launch givaudan_bringup demo.launch.py plan:=reachability/plans/task1_L08.json
    ros2 launch givaudan_bringup demo.launch.py plan:=... record:=/tmp/run_task1 gui:=false

The plan (reachability/plan_demo.py) names the robot's base pose, the tray and the vial.
This launch writes a demo copy of the shelf world in which that tray is dynamic, welds
to the robot's hands on command and has its vial loose (shelf_world's generator,
--grip / --pick-vial), spawns the robot there (robot_in_aisle.launch.py), bridges the
tray / vial poses and starts demo_runner.py; with record:=<dir> also record_run.py,
and the launch ends when the recording is written.
"""
import json
import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# plan robot -> (robot_in_aisle robot:=, Gazebo model name, hand frame -> Gazebo link). Gazebo
# merges links on fixed joints into their parent, so rby1's ee_left / ee_right only exist as
# part of link_*_arm_6: the welds go there (they keep the current relative pose anyway).
ROBOT_MODEL = {'rby1m': ('rby1', 'rby1', {'ee_left': 'link_left_arm_6', 'ee_right': 'link_right_arm_6'})}


def launch_setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    plan_file = os.path.abspath(arg('plan'))
    plan = json.load(open(plan_file))
    robot, model, links = ROBOT_MODEL[plan['robot']]
    tray, vial = plan['tray'], plan['vial']
    row, col = vial.split('_r')[1].split('c')

    world = arg('world')            # diagnosis: a prepared copy of the demo world
    if not world:
        world = os.path.join(tempfile.mkdtemp(prefix='givaudan_demo_'), 'shelf_aisle_demo.sdf')
    generator = os.path.join(get_package_prefix('shelf_world'), 'lib', 'shelf_world', 'generate_world.py')
    if not arg('world'):
        subprocess.run(['python3', generator, '-o', world,
                        '--grip', f"{tray}={model}:{links['ee_left']},{links['ee_right']}",
                        '--pick-vial', f"{tray}:{int(row)}:{int(col)}={model}:{links['ee_right']}"], check=True)

    base = plan['base']
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('givaudan_bringup'), 'launch',
                                                   'robot_in_aisle.launch.py')),
        launch_arguments={'robot': robot, 'world': world, 'gui': arg('gui'), 'x': str(base['x']),
                          'y': str(base['y']), 'yaw': str(base['yaw'])}.items())
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='demo_bridge', output='log',
                  arguments=[f'/model/{m}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose' for m in (tray, vial)],
                  parameters=[{'use_sim_time': True}])
    runner = Node(package='givaudan_bringup', executable='demo_runner.py', output='screen',
                  parameters=[{'plan': plan_file, 'speed': float(arg('speed')), 'use_sim_time': True,
                               'gz_link_ee_left': links['ee_left'], 'gz_link_ee_right': links['ee_right']}])
    actions = [sim, bridge, runner]
    if arg('record'):
        recorder = Node(package='givaudan_bringup', executable='record_run.py', output='screen',
                        parameters=[{'out_dir': os.path.abspath(arg('record')), 'world_file': world,
                                     'plan_file': plan_file, 'run_id': f"task{plan['task']}",
                                     'objects': [tray, vial], 'use_sim_time': True}])
        actions += [recorder, RegisterEventHandler(OnProcessExit(target_action=recorder,
                                                                 on_exit=[EmitEvent(event=Shutdown())]))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('plan', description='plan JSON from reachability/plan_demo.py'),
        DeclareLaunchArgument('record', default_value='', description='directory to record the run into'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('speed', default_value='1.0', description='>1 runs the plan faster'),
        DeclareLaunchArgument('world', default_value='', description='diagnosis: use this demo world file as is'),
        OpaqueFunction(function=launch_setup),
    ])
