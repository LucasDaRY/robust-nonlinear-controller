"""Subscribe to std_msgs/String messages on the chatter topic."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    """Log every String received on chatter."""

    def __init__(self) -> None:
        super().__init__("listener")
        self._subscription = self.create_subscription(
            String, "chatter", self._on_message, 10
        )

    def _on_message(self, message: String) -> None:
        self.get_logger().info(f'I heard: "{message.data}"')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Listener()
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
