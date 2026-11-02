import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rnc_demo = get_package_share_directory("rnc_demo")
    turtlebot3_carto = get_package_share_directory("turtlebot3_cartographer") # <-- NOUVEAU
    
    smc_config = os.path.join(rnc_demo, "config", "smc_params.yaml")
    rviz_config_path = os.path.join(rnc_demo, "rviz", "trajectory.rviz")

    cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_carto, "launch", "cartographer.launch.py")
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

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
        cartographer_cmd,
        rviz2_cmd,
        odom_node,
        trajectory_node,
        smc_node,
    ])