"""Launch Gazebo server and client with command line arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_name = "ugv_swarm"

    default_world = ""

    arg_world = DeclareLaunchArgument(
        "world", default_value=default_world, description="gazebo World"
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "launch_gazebo.launch.py",
            )
        ),
        launch_arguments={"world": world, "verbose": verbose, "gui": gui}.items(),
    )

    default_entity = "ugv_swarm"

    arg_entity = DeclareLaunchArgument(
        "entity", default_value=default_entity, description="robot name"
    )

    arg_x_pose = DeclareLaunchArgument(
        "x_pose", default_value="0.0", description="x pose of robot"
    )

    arg_y_pose = DeclareLaunchArgument(
        "y_pose", default_value="0.0", description="y pose of robot"
    )

    arg_yaw = DeclareLaunchArgument(
        "yaw", default_value="0.0", description="yaw of robot"
    )

    entity = LaunchConfiguration("entity")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    yaw = LaunchConfiguration("yaw")

    default_namespace = ""

    arg_namespace = DeclareLaunchArgument(
        "namespace", default_value=default_namespace, description="robot namespace"
    )

    namespace = LaunchConfiguration("namespace")

    spawn_bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "spawn_bot.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "entity": entity,
            "x_pose": x_pose,
            "y_pose": y_pose,
            "yaw": yaw,
        }.items(),
    )

    return LaunchDescription(
        [
            arg_namespace,
            arg_entity,
            arg_x_pose,
            arg_y_pose,
            arg_yaw,
            spawn_bot,
            arg_world,
            arg_verbose,
            arg_gui,
            gazebo,
        ]
    )
