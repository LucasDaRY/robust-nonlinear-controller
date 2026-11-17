import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    rnc_demo = get_package_share_directory("rnc_demo")
    turtlebot3_carto = get_package_share_directory("turtlebot3_cartographer")

    smc_config = os.path.join(rnc_demo, "config", "smc_params.yaml")
    slam_config = os.path.join(rnc_demo, "config", "slam_params.yaml")
    rviz_config_path = os.path.join(rnc_demo, "rviz", "trajectory.rviz")

    # CARTOGRAPHER — use_sim_time MUST be false on the real robot
    # use_rviz=false because we launch our own RViz with a custom config below
    cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_rviz': 'false',
        }.items()
    )

    # RViz with our custom config (delayed so Cartographer TF is ready)
    rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # Control nodes — delayed so Cartographer has time to publish map → odom TF
    # before odom_node starts querying map → base_footprint
    odom_node = Node(
        package="rnc_demo", executable="odom_node", output="screen",
        parameters=[{'use_sim_time': False}]
    )
    trajectory_node = Node(
        package="rnc_demo", executable="trajectory_node", output="screen",
        parameters=[{'use_sim_time': False}]
    )
    smc_node = Node(
        package="rnc_demo", executable="smc_node",
        parameters=[smc_config, {'use_sim_time': False}],
        output="screen"
    )

    delayed_control_nodes = TimerAction(
        period=8.0,  # Give Cartographer time to initialise and publish the TF map→odom
        actions=[odom_node, trajectory_node, smc_node]
    )

    return LaunchDescription([
        cartographer_cmd,
        rviz2_cmd,
        delayed_control_nodes,
    ])