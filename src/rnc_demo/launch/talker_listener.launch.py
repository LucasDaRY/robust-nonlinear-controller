"""Start the chatter publisher and subscriber together."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="rnc_demo",
                executable="talker",
                name="talker",
                output="screen",
            ),
            Node(
                package="rnc_demo",
                executable="listener",
                name="listener",
                output="screen",
            ),
        ]
    )
