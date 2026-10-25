import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rnc_demo = get_package_share_directory("rnc_demo")
    smc_config = os.path.join(rnc_demo, "config", "smc_params.yaml")
    rviz_config_path = os.path.join(rnc_demo, "rviz", "trajectory.rviz")

    rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    odom_node = Node(package="rnc_demo", executable="odom_node", output="screen")
    trajectory_node = Node(package="rnc_demo", executable="trajectory_node", output="screen")
    smc_node = Node(package="rnc_demo", executable="smc_node", parameters=[smc_config], output="screen")

    return LaunchDescription([
        rviz2_cmd,
        odom_node,
        trajectory_node,
        smc_node,
    ])