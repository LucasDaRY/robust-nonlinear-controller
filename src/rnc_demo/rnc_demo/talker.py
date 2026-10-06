"""Publish std_msgs/String messages on the chatter topic."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    """Publish one String per second on chatter."""

    def __init__(self) -> None:
        super().__init__("talker")
        self._publisher = self.create_publisher(String, "chatter", 10)
        self._count = 0
        self._timer = self.create_timer(1.0, self._on_timer)

    def _on_timer(self) -> None:
        message = String()
        message.data = f"Hello, TurtleBot3 {self._count}"
        self._publisher.publish(message)
        self.get_logger().info(f'Publishing: "{message.data}"')
        self._count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
