"""Spawn a TurtleBot3 Burger on the 1 m table world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    turtlebot3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    gazebo_ros = get_package_share_directory("gazebo_ros")
    rnc_demo = get_package_share_directory("rnc_demo")

    model = os.environ.get("TURTLEBOT3_MODEL", "burger")
    urdf_path = os.path.join(
        turtlebot3_gazebo, "models", f"turtlebot3_{model}", "model.sdf"
    )
    world = os.path.join(rnc_demo, "worlds", "table_1m.world")

    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gzclient.launch.py")
        )
    )
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    spawn_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            model,
            "-file",
            urdf_path,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo clock.",
            ),
            DeclareLaunchArgument(
                "x_pose",
                default_value="0.0",
                description="Burger spawn position on x, in metres.",
            ),
            DeclareLaunchArgument(
                "y_pose",
                default_value="0.0",
                description="Burger spawn position on y, in metres.",
            ),
            DeclareLaunchArgument(
                "z_pose",
                default_value="0.76",
                description="Burger spawn height. The table top is at 0.75 m.",
            ),
            gzserver_cmd,
            gzclient_cmd,
            robot_state_publisher_cmd,
            spawn_cmd,
        ]
    )
