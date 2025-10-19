"""Launch Gazebo and spawn multiple robots based on a YAML config."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def load_robot_params(yaml_file):
    with open(yaml_file, "r") as f:
        data = yaml.safe_load(f)
    return data.get("robots", [])


def launch_setup(context, *args, **kwargs):
    package_name = "ugv_swarm"

    world = LaunchConfiguration("world").perform(context)
    verbose = LaunchConfiguration("verbose").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    robots_config = LaunchConfiguration("robots_config").perform(context)

    # Gazebo server + GUI
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

    # Load robots from robots_config
    robots = load_robot_params(robots_config)

    spawn_actions = []
    for robot in robots:
        name = str(robot.get("name", "robot"))
        x = str(robot.get("x", 0.0))
        y = str(robot.get("y", 0.0))
        yaw = str(robot.get("yaw", 0.0))

        spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "spawn_bot.launch.py",
                )
            ),
            launch_arguments={
                "entity": name,
                "namespace": name,
                "x_pose": x,
                "y_pose": y,
                "yaw": yaw,
            }.items(),
        )

        spawn_actions.append(spawn)

    return [gazebo, *spawn_actions]


def generate_launch_description():
    package_name = "ugv_swarm"

    # Gazebo args
    arg_world = DeclareLaunchArgument(
        "world", default_value="", description="Gazebo world file"
    )

    arg_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Enable verbose Gazebo output (-v4)",
    )

    arg_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Enable Gazebo GUI visualization"
    )

    default_robots_config = os.path.join(
        get_package_share_directory(package_name), "config", "multi_bot.yaml"
    )

    arg_robots_config = DeclareLaunchArgument(
        "robots_config",
        default_value=default_robots_config,
        description="Path to YAML file containing robots' names and poses",
    )

    return LaunchDescription(
        [
            arg_world,
            arg_verbose,
            arg_gui,
            arg_robots_config,
            OpaqueFunction(function=launch_setup),
        ]
    )
