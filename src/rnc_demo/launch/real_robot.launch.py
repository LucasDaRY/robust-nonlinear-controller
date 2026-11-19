import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")
    rnc_demo = get_package_share_directory("rnc_demo")

    smc_config = os.path.join(rnc_demo, "config", "smc_params.yaml")
    slam_config = os.path.join(rnc_demo, "config", "slam_params.yaml")
    rviz_config_path = os.path.join(rnc_demo, "rviz", "trajectory.rviz")

    # ONLINE SLAM (publie map -> odom à 50 Hz, sans carte préalable)
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": slam_config,
            "use_sim_time": "false",
        }.items(),
    )

    # RVIZ2
    rviz2_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    # CONTROL NODES
    odom_node = Node(
        package="rnc_demo",
        executable="odom_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )
    trajectory_node = Node(
        package="rnc_demo",
        executable="trajectory_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )
    smc_node = Node(
        package="rnc_demo",
        executable="smc_node",
        parameters=[smc_config, {"use_sim_time": False}],
        output="screen",
    )

    delayed_control_nodes = TimerAction(
        period=3.0,
        actions=[odom_node, trajectory_node, smc_node],
    )

    return LaunchDescription([
        slam_cmd,
        rviz2_cmd,
        delayed_control_nodes,
    ])