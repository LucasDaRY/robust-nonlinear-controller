import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    rnc_demo = get_package_share_directory("rnc_demo")

    smc_config = os.path.join(rnc_demo, "config", "smc_params.yaml")
    ekf_config = os.path.join(rnc_demo, "config", "ekf.yaml")
    rviz_config_path = os.path.join(rnc_demo, "rviz", "trajectory.rviz")

    # Aligne la convention LiDAR TurtleBot3 [0..2pi] avec la convention rf2o [-pi..pi]
    tf_base_rf2o = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_base_rf2o",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "3.14159265359",
            "--frame-id", "base_footprint", "--child-frame-id", "base_footprint_rf2o",
        ],
        parameters=[{"use_sim_time": False}],
    )

    # Odométrie LiDAR scan-à-scan (souscrit nativement à /scan en Best Effort)
    rf2o_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[{
            "laser_scan_topic": "/scan",
            "odom_topic": "/odom_rf2o",
            "publish_tf": False,
            "base_frame_id": "base_footprint_rf2o",
            "odom_frame_id": "map",
            "init_pose_from_topic": "",
            "freq": 5.0,
            "use_sim_time": False,
        }],
    )

    # Fusion EKF (rf2o x/y + IMU yaw -> publie map -> odom à 50 Hz)
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": False}],
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

    # NŒUDS DE COMMANDE SMC
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

    delayed_nodes_cmd = TimerAction(
        period=8.0,
        actions=[trajectory_node, smc_node],
    )

    return LaunchDescription([
        tf_base_rf2o,
        rf2o_node,
        ekf_node,
        rviz2_cmd,
        delayed_nodes_cmd,
    ])