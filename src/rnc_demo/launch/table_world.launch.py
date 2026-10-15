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
    world = os.path.join(rnc_demo, "worlds", "table_1m.world")
    smc_config = os.path.join(rnc_demo, "config", "smc_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")

    # Évite les requêtes réseau lentes sur WSL
    set_env_gazebo = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world,
            "extra_gazebo_args": "-s libgazebo_ros_factory.so",
        }.items(),
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
            "-entity", model,
            "-file", urdf_path,
            "-x", x_pose,
            "-y", y_pose,
            "-z", z_pose,
        ],
        output="screen",
    )

    # 1. Spawn du robot après 10s
    delayed_spawn_cmd = TimerAction(
        period=10.0,
        actions=[spawn_cmd]
    )

    # 2. Nœuds de contrôle
    odom_node = Node(
        package="rnc_demo",
        executable="odom_node",
        output="screen",
    )

    trajectory_node = Node(
        package="rnc_demo",
        executable="trajectory_node",
        output="screen",
    )

    smc_node = Node(
        package="rnc_demo",
        executable="smc_node",
        parameters=[smc_config],
        output="screen",
    )

    # Lancement de la boucle de contrôle à 13s (quand le robot est posé sur la table)
    delayed_nodes_cmd = TimerAction(
        period=13.0,
        actions=[odom_node, trajectory_node, smc_node]
    )

    return LaunchDescription(
        [
            set_env_gazebo,
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use Gazebo clock"),
            DeclareLaunchArgument("x_pose", default_value="0.0", description="Burger spawn position on x"),
            DeclareLaunchArgument("y_pose", default_value="0.0", description="Burger spawn position on y"),
            DeclareLaunchArgument("z_pose", default_value="0.76", description="Burger spawn height"),
            gzserver_cmd,
            gzclient_cmd,
            robot_state_publisher_cmd,
            delayed_spawn_cmd,
            delayed_nodes_cmd,
        ]
    )