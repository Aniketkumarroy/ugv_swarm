"""Launch Gazebo server and client with command line arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

# from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution


from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_name = "ugv_swarm"

    pkg_gazebo_ros = "gazebo_ros"

    default_namespace = ""

    arg_namespace = DeclareLaunchArgument(
        "namespace", default_value=default_namespace, description="robot namespace"
    )

    namespace = LaunchConfiguration("namespace")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true", "namespace": namespace}.items(),
    )

    default_entity = "diff_drive_bot"

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

    # Run the spawner node from the pkg_gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    robot_description_topic = PythonExpression(
        [
            "'/robot_description' if '",
            namespace,
            "' == '' else '",
            namespace,
            "' + '/robot_description'",
        ]
    )
    spawn_entity = Node(
        package=pkg_gazebo_ros,
        executable="spawn_entity.py",
        name=[namespace, TextSubstitution(text="_state_publisher")],
        arguments=[
            "-topic",
            robot_description_topic,
            "-entity",
            entity,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            "0.0",
            "-Y",
            yaw,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            arg_namespace,
            rsp,
            arg_entity,
            arg_x_pose,
            arg_y_pose,
            arg_yaw,
            spawn_entity,
        ]
    )
