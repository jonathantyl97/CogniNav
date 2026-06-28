"""Replay a recorded live-rig rosbag through the CogniNav stack."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("cogninav_bringup")
    viz_params = os.path.join(pkg_share, "config", "cogninav_viz.yaml")

    bag_path = LaunchConfiguration("bag_path")
    rate = LaunchConfiguration("rate")
    bag_play_delay = LaunchConfiguration("bag_play_delay")
    bag_loop = LaunchConfiguration("bag_loop")
    use_viz = LaunchConfiguration("use_viz")
    use_pangolin_viewer = LaunchConfiguration("use_pangolin_viewer")
    use_vslam = LaunchConfiguration("use_vslam")
    use_depth = LaunchConfiguration("use_depth")
    use_lanes = LaunchConfiguration("use_lanes")
    use_sim_time = LaunchConfiguration("use_sim_time")
    show_stereo_depth = LaunchConfiguration("show_stereo_depth")

    bag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag_path, "--clock", "-r", rate],
        output="screen",
        condition=UnlessCondition(bag_loop),
    )
    bag_play_loop = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag_path, "--clock", "-r", rate, "--loop"],
        output="screen",
        condition=IfCondition(bag_loop),
    )

    def launch_setup(context, *args, **kwargs):
        rig_name = LaunchConfiguration("rig").perform(context)
        params_file = os.path.join(pkg_share, "config", f"{rig_name}.yaml")
        if not os.path.isfile(params_file):
            raise RuntimeError(
                f"Unknown rig '{rig_name}' — expected config at {params_file}"
            )
        pangolin = (
            LaunchConfiguration("use_pangolin_viewer").perform(context).lower() == "true"
        )
        return [
            Node(
                package="cogninav_vslam",
                executable="orb_slam3_node",
                name="cogninav_vslam",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time, "use_orb_viewer": pangolin},
                ],
                output="screen",
                condition=IfCondition(use_vslam),
            ),
            Node(
                package="cogninav_depth",
                executable="stereo_depth",
                name="cogninav_depth",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(use_depth),
            ),
            Node(
                package="cogninav_lanes",
                executable="corridor_monitor",
                name="cogninav_corridor_monitor",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(use_lanes),
            ),
            TimerAction(period=bag_play_delay, actions=[bag_play, bag_play_loop]),
            Node(
                package="cogninav_viz",
                executable="iridescence_viewer",
                name="cogninav_viz",
                parameters=[
                    viz_params,
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "show_stereo_depth": show_stereo_depth,
                    },
                ],
                output="screen",
                condition=IfCondition(use_viz),
            ),
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rig",
                default_value="realsense_d455",
                description="Rig preset: realsense_d455 | zed2",
            ),
            DeclareLaunchArgument(
                "bag_path",
                default_value="/root/Downloads/cogninav/realsense_d455_warehouse_aisle1",
            ),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("bag_play_delay", default_value="10.0"),
            DeclareLaunchArgument("bag_loop", default_value="true"),
            DeclareLaunchArgument("use_viz", default_value="true"),
            DeclareLaunchArgument("use_pangolin_viewer", default_value="false"),
            DeclareLaunchArgument("use_vslam", default_value="true"),
            DeclareLaunchArgument("use_depth", default_value="true"),
            DeclareLaunchArgument("use_lanes", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("show_stereo_depth", default_value="true"),
            SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
            SetEnvironmentVariable(name="RMW_FASTRTPS_USE_SHM", value="0"),
            SetEnvironmentVariable(name="OMP_NUM_THREADS", value="4"),
            AppendEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value="/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
