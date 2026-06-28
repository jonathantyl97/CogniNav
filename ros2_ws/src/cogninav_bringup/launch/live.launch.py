from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("cogninav_bringup")
    use_viz = LaunchConfiguration("use_viz")
    use_pangolin_viewer = LaunchConfiguration("use_pangolin_viewer")
    use_lanes = LaunchConfiguration("use_lanes")
    use_depth = LaunchConfiguration("use_depth")
    use_vslam = LaunchConfiguration("use_vslam")
    viz_params = os.path.join(pkg_share, "config", "cogninav_viz.yaml")

    def launch_setup(context, *args, **kwargs):
        rig_name = LaunchConfiguration("rig").perform(context)
        params_file = os.path.join(pkg_share, "config", f"{rig_name}.yaml")
        if not os.path.isfile(params_file):
            raise RuntimeError(
                f"Unknown rig '{rig_name}' — expected config at {params_file}"
            )
        pangolin = LaunchConfiguration("use_pangolin_viewer").perform(context).lower() == "true"
        return [
            Node(
                package="cogninav_vslam",
                executable="orb_slam3_node",
                name="cogninav_vslam",
                parameters=[params_file, {"use_orb_viewer": pangolin}],
                output="screen",
                condition=IfCondition(use_vslam),
            ),
            Node(
                package="cogninav_depth",
                executable="stereo_depth",
                name="cogninav_depth",
                parameters=[params_file],
                output="screen",
                condition=IfCondition(use_depth),
            ),
            Node(
                package="cogninav_lanes",
                executable="corridor_monitor",
                name="cogninav_corridor_monitor",
                parameters=[params_file],
                output="screen",
                condition=IfCondition(use_lanes),
            ),
            Node(
                package="cogninav_viz",
                executable="iridescence_viewer",
                name="cogninav_viz",
                parameters=[viz_params],
                output="screen",
                condition=IfCondition(use_viz),
            ),
        ]

    return LaunchDescription(
        [
            AppendEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value="/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib",
            ),
            DeclareLaunchArgument(
                "rig",
                default_value="realsense_d455",
                description="Rig preset: realsense_d455 | zed2",
            ),
            DeclareLaunchArgument("use_vslam", default_value="true"),
            DeclareLaunchArgument(
                "use_viz",
                default_value="true",
                description="Iridescence viewer (requires DISPLAY / X11)",
            ),
            DeclareLaunchArgument(
                "use_pangolin_viewer",
                default_value="false",
                description="ORB-SLAM3 Pangolin viewer (mutually exclusive with use_viz)",
            ),
            DeclareLaunchArgument("use_lanes", default_value="true"),
            DeclareLaunchArgument("use_depth", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
