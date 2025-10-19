"""Launch Gazebo server and client with command line arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_gazebo_ros = "gazebo_ros"

    default_world = ""

    arg_world = DeclareLaunchArgument(
        "world", default_value=default_world, description="gazebo Simulation World"
    )

    world = LaunchConfiguration("world")

    arg_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Enable verbose Gazebo output (-v4)",
    )

    verbose = LaunchConfiguration("verbose")

    arg_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Enable Simulator Visualization",
    )

    gui = LaunchConfiguration("gui")

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(pkg_gazebo_ros),
                    "launch",
                    "gzserver.launch.py",
                )
            ]
        ),
        launch_arguments={"world": world, "verbose": verbose}.items(),
    )

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(pkg_gazebo_ros),
                    "launch",
                    "gzclient.launch.py",
                )
            ]
        ),
        condition=IfCondition(gui),
    )

    return LaunchDescription(
        [
            arg_world,
            arg_verbose,
            arg_gui,
            gz_server,
            gz_gui,
        ]
    )
