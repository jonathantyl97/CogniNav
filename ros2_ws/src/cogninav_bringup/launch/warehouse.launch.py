from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("cogninav_bringup")
    warehouse_params = PathJoinSubstitution([pkg, "config", "warehouse_torwic.yaml"])
    viz_params = PathJoinSubstitution([pkg, "config", "cogninav_viz.yaml"])
    viz_light_params = PathJoinSubstitution([pkg, "config", "warehouse_viz_torwic.yaml"])

    bag_path = LaunchConfiguration("bag_path")
    rate = LaunchConfiguration("rate")
    bag_play_delay = LaunchConfiguration("bag_play_delay")
    use_viz = LaunchConfiguration("use_viz")
    use_pangolin_viewer = LaunchConfiguration("use_pangolin_viewer")
    use_vslam = LaunchConfiguration("use_vslam")
    use_depth = LaunchConfiguration("use_depth")
    use_lanes = LaunchConfiguration("use_lanes")
    use_sim_time = LaunchConfiguration("use_sim_time")
    show_stereo_depth = LaunchConfiguration("show_stereo_depth")
    bag_loop = LaunchConfiguration("bag_loop")

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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="/root/Downloads/warehouse/aisle_cw_run_1_ros2",
            ),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument(
                "bag_play_delay",
                default_value="12.0",
                description="Seconds to wait before bag play (ORB vocab load)",
            ),
            DeclareLaunchArgument("use_viz", default_value="true"),
            DeclareLaunchArgument(
                "use_pangolin_viewer",
                default_value="false",
                description="ORB-SLAM3 built-in Pangolin viewer (mutually exclusive with use_viz)",
            ),
            DeclareLaunchArgument("use_vslam", default_value="true"),
            DeclareLaunchArgument("use_depth", default_value="false"),
            DeclareLaunchArgument("use_lanes", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("show_stereo_depth", default_value="false"),
            DeclareLaunchArgument(
                "bag_loop",
                default_value="false",
                description="Loop bag playback (use for short benchmark bags)",
            ),
            SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
            SetEnvironmentVariable(name="RMW_FASTRTPS_USE_SHM", value="0"),
            SetEnvironmentVariable(name="OMP_NUM_THREADS", value="4"),
            AppendEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value="/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib",
            ),
            Node(
                package="image_transport",
                executable="republish",
                name="republish_left",
                arguments=["compressed", "raw"],
                parameters=[
                    {"in_transport": "compressed", "out_transport": "raw"},
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("in/compressed", "/left_azure/rgb/image_raw/compressed"),
                    ("out", "/cam0/image_raw"),
                ],
                output="screen",
            ),
            Node(
                package="image_transport",
                executable="republish",
                name="republish_right",
                arguments=["compressed", "raw"],
                parameters=[
                    {"in_transport": "compressed", "out_transport": "raw"},
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("in/compressed", "/right_azure/rgb/image_raw/compressed"),
                    ("out", "/cam1/image_raw"),
                ],
                output="screen",
            ),
            Node(
                package="cogninav_vslam",
                executable="orb_slam3_node",
                name="cogninav_vslam",
                parameters=[
                    warehouse_params,
                    viz_light_params,
                    {"use_sim_time": use_sim_time, "use_orb_viewer": use_pangolin_viewer},
                ],
                output="screen",
                condition=IfCondition(use_vslam),
            ),
            Node(
                package="cogninav_depth",
                executable="stereo_depth",
                name="cogninav_depth",
                parameters=[warehouse_params, viz_light_params, {"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(use_depth),
            ),
            Node(
                package="cogninav_lanes",
                executable="corridor_monitor",
                name="cogninav_corridor_monitor",
                parameters=[warehouse_params, viz_light_params, {"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(use_lanes),
            ),
            TimerAction(
                period=bag_play_delay,
                actions=[bag_play, bag_play_loop],
            ),
            Node(
                package="cogninav_viz",
                executable="iridescence_viewer",
                name="cogninav_viz",
                parameters=[
                    viz_params,
                    warehouse_params,
                    viz_light_params,
                    {
                        "use_sim_time": use_sim_time,
                        "show_stereo_depth": show_stereo_depth,
                    },
                ],
                output="screen",
                condition=IfCondition(use_viz),
            ),
        ]
    )
