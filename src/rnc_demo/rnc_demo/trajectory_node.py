#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.pub = self.create_publisher(Float64MultiArray, '/desired_trajectory', 10)
        self.timer = self.create_timer(0.05, self.timer_cb) # 20 Hz
        self.t0 = time.time()

    def timer_cb(self):
        t = time.time() - self.t0
        
        # Position désirée
        x_d = 0.5 * math.cos(t / 10.0) + 0.5
        y_d = 0.5 * math.sin(t / 10.0) + 0.5
        
        # Vitesse désirée (dérivée première)
        x_d_dot = -0.05 * math.sin(t / 10.0)
        y_d_dot = 0.05 * math.cos(t / 10.0)
        
        # Accélération désirée (dérivée seconde)
        x_d_ddot = -0.005 * math.cos(t / 10.0)
        y_d_ddot = -0.005 * math.sin(t / 10.0)

        msg = Float64MultiArray()
        # [x, y, x_dot, y_dot, x_ddot, y_ddot]
        msg.data = [x_d, y_d, x_d_dot, y_d_dot, x_d_ddot, y_d_ddot]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()