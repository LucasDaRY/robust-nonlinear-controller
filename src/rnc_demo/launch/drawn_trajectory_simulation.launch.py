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
    rviz_config_path = os.path.join(rnc_demo, "rviz", "trajectory.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
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
            "-entity", model, "-file", urdf_path,
            "-x", "0.0", "-y", "0.0", "-z", "0.01", "-Y", "0.0",
        ],
        output="screen",
    )
    delayed_spawn_cmd = TimerAction(period=5.0, actions=[spawn_cmd])

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

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
    )

    rviz2_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    delayed_rviz_cmd = TimerAction(period=6.0, actions=[rviz2_cmd])

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

    return LaunchDescription([
        set_env_gazebo,
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("yaw_pose", default_value="0.0"),
        gzserver_cmd,
        robot_state_publisher_cmd,
        delayed_spawn_cmd,
        tf_base_rf2o,
        rf2o_node,
        ekf_node,
        delayed_rviz_cmd,
        delayed_nodes_cmd,
    ])