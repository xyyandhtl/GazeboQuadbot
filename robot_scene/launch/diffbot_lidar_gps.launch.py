"""Gazebo Classic: diff_bot + Mid-360 (mid360_simulation, same as go2w)."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_scene")
    # 与 go2w_description 一致：让 Gazebo 找到 libmid360_plugin.so
    mid360_lib = os.path.join(get_package_prefix("mid360_simulation"), "lib")
    prev_plugins = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    os.environ["GAZEBO_PLUGIN_PATH"] = (
        mid360_lib if not prev_plugins else f"{mid360_lib}:{prev_plugins}"
    )

    default_world = os.path.join(pkg_share, "worlds", "bigHHH.world")
    xacro_file = os.path.join(pkg_share, "xacro", "diff_bot.xacro")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Forward to robot_state_publisher / spawn.",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo .world under this package or absolute path.",
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="diffbot",
        description="Gazebo spawn entity name (-entity), not TF prefix.",
    )
    declare_tf_prefix = DeclareLaunchArgument(
        "tf_prefix",
        default_value="",
        description=(
            "URDF xacro arg prefix for links/joints/odom TF. "
            "Empty (default) => odom, base_link, livox, ... "
            "Multi-robot example: tf_prefix:=diffbot_"
        ),
    )
    declare_spawn_x = DeclareLaunchArgument("spawn_x", default_value="0.0")
    declare_spawn_y = DeclareLaunchArgument("spawn_y", default_value="0.0")
    declare_spawn_z = DeclareLaunchArgument("spawn_z", default_value="0.1")

    robot_description = Command(
        [
            "xacro ",
            xacro_file,
            " prefix:=",
            LaunchConfiguration("tf_prefix"),
        ]
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("gazebo_ros"),
                "/launch/gzserver.launch.py",
            ]
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "extra_gazebo_args": " --verbose",
        }.items(),
    )

    gzclient = ExecuteProcess(cmd=["gzclient", "--verbose"], output="screen")

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(robot_description, value_type=str),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            LaunchConfiguration("robot_name"),
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("spawn_x"),
            "-y",
            LaunchConfiguration("spawn_y"),
            "-z",
            LaunchConfiguration("spawn_z"),
        ],
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_world,
            declare_robot_name,
            declare_tf_prefix,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            gzserver,
            gzclient,
            rsp,
            spawn,
        ]
    )
