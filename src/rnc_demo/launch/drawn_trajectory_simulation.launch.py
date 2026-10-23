import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
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
    
    world = os.path.join(turtlebot3_gazebo, "worlds", "empty_world.world")
    smc_config = os.path.join(rnc_demo, "config", "smc_params.yaml")
    rviz_config_path = os.path.join(rnc_demo, "rviz", "trajectory.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")

    set_env_gazebo = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

    # HEADLESS GAZEBO 
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world,
            "extra_gazebo_args": "-s libgazebo_ros_factory.so",
        }.items(),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # SPAWN DEFINITION
    spawn_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", model, "-file", urdf_path,
            "-x", "0.0", "-y", "0.0", "-z", "0.01", "-Y", "1.5708",
        ],
        output="screen",
    )
    delayed_spawn_cmd = TimerAction(period=5.0, actions=[spawn_cmd])

    # RVIZ2
    rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    delayed_rviz_cmd = TimerAction(period=6.0, actions=[rviz2_cmd])

    # CONTROL NODES
    odom_node = Node(package="rnc_demo", executable="odom_node", output="screen")
    trajectory_node = Node(package="rnc_demo", executable="trajectory_node", output="screen")
    smc_node = Node(package="rnc_demo", executable="smc_node", parameters=[smc_config], output="screen")

    delayed_nodes_cmd = TimerAction(
        period=8.0,
        actions=[odom_node, trajectory_node, smc_node]
    )

    return LaunchDescription(
        [
            set_env_gazebo,
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("yaw_pose", default_value="1.5708"),
            
            gzserver_cmd,
            robot_state_publisher_cmd,
            delayed_spawn_cmd,
            delayed_rviz_cmd,
            delayed_nodes_cmd,
        ]
    )