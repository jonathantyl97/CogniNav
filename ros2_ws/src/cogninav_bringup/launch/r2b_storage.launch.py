from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("cogninav_bringup")
    r2b_params = PathJoinSubstitution([pkg, "config", "warehouse_r2b.yaml"])
    viz_params = PathJoinSubstitution([pkg, "config", "cogninav_viz.yaml"])

    bag_path = LaunchConfiguration("bag_path")
    rate = LaunchConfiguration("rate")
    use_viz = LaunchConfiguration("use_viz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="/root/Downloads/warehouse/r2b_storage",
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
                package="cogninav_vslam",
                executable="orb_slam3_node",
                name="cogninav_vslam",
                parameters=[r2b_params, {"use_sim_time": use_sim_time}],
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
