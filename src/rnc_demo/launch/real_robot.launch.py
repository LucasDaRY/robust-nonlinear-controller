import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    rnc_demo = get_package_share_directory("rnc_demo")

    smc_config = os.path.join(rnc_demo, "config", "smc_params.yaml")
    imu_odom_config = os.path.join(rnc_demo, "config", "imu_odom_params.yaml")
    rf2o_config = os.path.join(rnc_demo, "config", "rf2o_params.yaml")
    rviz_config_path = os.path.join(rnc_demo, "rviz", "trajectory.rviz")

    # Pont QoS : /scan (Best Effort) -> /scan_reliable (Reliable)
    scan_relay_node = Node(
        package="rnc_demo",
        executable="scan_relay",
        name="scan_relay_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
        parameters=[{"use_sim_time": False}],
    )

    rf2o_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[rf2o_config, {"use_sim_time": False}],
    )

    # RVIZ2 (remappé sur /scan_reliable pour éviter l'incompatibilité QoS sur /scan)
    rviz2_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": False}],
        remappings=[("/scan", "/scan_reliable")],
        output="screen",
    )

    # CONTROL NODES
    odom_node = Node(
        package="rnc_demo",
        executable="odom_node",
        output="screen",
        parameters=[imu_odom_config, {"use_sim_time": False}],
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
        scan_relay_node,
        static_map_to_odom,
        rf2o_node,
        rviz2_cmd,
        delayed_control_nodes,
    ])