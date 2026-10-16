"""Start Gazebo Harmonic with the shelf-aisle world and bridge /clock to ROS 2."""
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("shelf_world")
    world = LaunchConfiguration("world")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value=PathJoinSubstitution([pkg, "worlds", "shelf_aisle.sdf"]),
            description="World file to load"),
        DeclareLaunchArgument(
            "extra_gz_args", default_value="",
            description="Extra gz sim arguments, e.g. '-s --headless-rendering' for no GUI"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([pkg, "models"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
            launch_arguments={"gz_args": [world, " -r ", LaunchConfiguration("extra_gz_args")], "on_exit_shutdown": "true"}.items()),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen"),
    ])
