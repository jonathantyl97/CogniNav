from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("cogninav_bringup")
    viz_params = PathJoinSubstitution([pkg, "config", "cogninav_viz.yaml"])
    vslam_params = PathJoinSubstitution([pkg, "config", "cogninav_vslam.yaml"])
    lanes_params = PathJoinSubstitution([pkg, "config", "cogninav_corridor.yaml"])
    depth_params = PathJoinSubstitution([pkg, "config", "cogninav_depth.yaml"])

    use_viz = LaunchConfiguration("use_viz")
    use_lanes = LaunchConfiguration("use_lanes")
    use_depth = LaunchConfiguration("use_depth")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_viz",
                default_value="true",
                description="Start Iridescence desktop viewer (requires DISPLAY / X11)",
            ),
            DeclareLaunchArgument(
                "use_lanes",
                default_value="true",
                description="Start lane corridor monitor (lanes + in-lane human/car)",
            ),
            DeclareLaunchArgument(
                "use_depth",
                default_value="true",
                description="Start stereo depth (OpenCV SGBM, left + right topics)",
            ),
            DeclareLaunchArgument(
                "viz_params",
                default_value=viz_params,
                description="Iridescence viewer parameters",
            ),
            DeclareLaunchArgument(
                "lanes_params",
                default_value=lanes_params,
                description="Lane / corridor monitor parameters",
            ),
            DeclareLaunchArgument(
                "depth_params",
                default_value=depth_params,
                description="Stereo depth parameters",
            ),
            DeclareLaunchArgument(
                "vslam_params",
                default_value=vslam_params,
                description="ORB-SLAM3 stereo wrapper parameters",
            ),
            # cogninav_vslam — stereo / stereo-inertial ORB-SLAM3 (Phase 1).
            # Node(
            #     package="cogninav_vslam",
            #     executable="orb_slam3_node",
            #     name="cogninav_vslam",
            #     parameters=[LaunchConfiguration("vslam_params")],
            #     output="screen",
            # ),
            Node(
                package="cogninav_depth",
                executable="stereo_depth",
                name="cogninav_depth",
                parameters=[LaunchConfiguration("depth_params")],
                output="screen",
                condition=IfCondition(use_depth),
            ),
            Node(
                package="cogninav_lanes",
                executable="corridor_monitor",
                name="cogninav_corridor_monitor",
                parameters=[LaunchConfiguration("lanes_params")],
                output="screen",
                condition=IfCondition(use_lanes),
            ),
            Node(
                package="cogninav_viz",
                executable="iridescence_viewer",
                name="cogninav_viz",
                parameters=[LaunchConfiguration("viz_params")],
                output="screen",
                condition=IfCondition(use_viz),
            ),
        ]
    )
