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

    world = os.path.join(rnc_demo, "worlds", "custom_room.world")
    smc_config = os.path.join(rnc_demo, "config", "smc_params.yaml")
    ekf_config = os.path.join(rnc_demo, "config", "ekf.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    yaw_pose = LaunchConfiguration("yaw_pose")

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
            "-Y", yaw_pose,
        ],
        output="screen",
    )

    delayed_spawn_cmd = TimerAction(
        period=5.0,
        actions=[spawn_cmd],
    )

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
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Odométrie LiDAR scan-à-scan
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
            "use_sim_time": use_sim_time,
        }],
    )

    # Fusion EKF (rf2o x/y + IMU yaw -> publie map -> odom à 50 Hz)
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
    )

    trajectory_node = Node(
        package="rnc_demo",
        executable="trajectory_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    smc_node = Node(
        package="rnc_demo",
        executable="smc_node",
        parameters=[smc_config, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    delayed_nodes_cmd = TimerAction(
        period=8.0,
        actions=[trajectory_node, smc_node],
    )

    return LaunchDescription(
        [
            set_env_gazebo,
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use Gazebo clock"),
            DeclareLaunchArgument("x_pose", default_value="0.0", description="Burger spawn position on x"),
            DeclareLaunchArgument("y_pose", default_value="0.0", description="Burger spawn position on y"),
            DeclareLaunchArgument("z_pose", default_value="0.01", description="Burger spawn height"),
            DeclareLaunchArgument("yaw_pose", default_value="0.0", description="Burger spawn orientation"),
            gzserver_cmd,
            gzclient_cmd,
            robot_state_publisher_cmd,
            delayed_spawn_cmd,
            tf_base_rf2o,
            rf2o_node,
            ekf_node,
            delayed_nodes_cmd,
        ]
    )