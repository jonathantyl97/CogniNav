from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("cogninav_bringup")
    warehouse_params = PathJoinSubstitution([pkg, "config", "warehouse_torwic.yaml"])
    viz_params = PathJoinSubstitution([pkg, "config", "cogninav_viz.yaml"])

    bag_path = LaunchConfiguration("bag_path")
    rate = LaunchConfiguration("rate")
    use_viz = LaunchConfiguration("use_viz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="/root/Downloads/warehouse/aisle_cw_run_1_ros2",
            ),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("use_viz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
            AppendEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value="/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib",
            ),
            Node(
                package="image_transport",
                executable="republish",
                name="republish_left",
                arguments=["compressed", "raw"],
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
                parameters=[warehouse_params, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path, "--clock", "-r", rate],
                output="screen",
            ),
            Node(
                package="cogninav_viz",
                executable="iridescence_viewer",
                name="cogninav_viz",
                parameters=[viz_params, {"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(use_viz),
            ),
        ]
    )
