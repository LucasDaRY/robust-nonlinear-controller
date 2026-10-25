#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose2D
import math
import time
import sympy as sp

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        
        # Crontrol
        self.pub_cmd = self.create_publisher(Float64MultiArray, '/desired_trajectory', 10)
        
        # Draw
        self.pub_path = self.create_publisher(Path, '/desired_path', 10)

        # Verify robot spawn
        self.sub_pose = self.create_subscription(Pose2D, '/robot_pose', self.pose_cb, 10)
        self.robot_ready = False
        
        self.timer_cmd = self.create_timer(0.05, self.timer_cb) # 20 Hz control
        self.timer_path = self.create_timer(1.0, self.publish_path) # 1 Hz draw
        
        self.t0 = None

        # Trajectory Parameters
        self.omega = 0.1  # Main pulsation
        self.t_max = 2.0 * math.pi / self.omega  # Loop duration
        t_sym = sp.Symbol('t')

        # Parametric curve definition
        x_sym = 0.5 * sp.sin(2.0 * self.omega * t_sym)
        y_sym = 0.5 * sp.cos(self.omega * t_sym) - 0.7

        # Automatic derivatives
        x_dot_sym = sp.diff(x_sym, t_sym)
        y_dot_sym = sp.diff(y_sym, t_sym)
        x_ddot_sym = sp.diff(x_dot_sym, t_sym)
        y_ddot_sym = sp.diff(y_dot_sym, t_sym)

        # Convert to function
        self.get_x = sp.lambdify(t_sym, x_sym, 'math')
        self.get_y = sp.lambdify(t_sym, y_sym, 'math')
        self.get_dx = sp.lambdify(t_sym, x_dot_sym, 'math')
        self.get_dy = sp.lambdify(t_sym, y_dot_sym, 'math')
        self.get_ddx = sp.lambdify(t_sym, x_ddot_sym, 'math')
        self.get_ddy = sp.lambdify(t_sym, y_ddot_sym, 'math')

    def pose_cb(self, msg):
        self.robot_ready = True

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(1000):
            t_sim = i * 0.5 # second per point
            # Automatic stop
            if t_sim > self.t_max:
                break

            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = self.get_x(t_sim)
            pose.pose.position.y = self.get_y(t_sim)
            pose.pose.position.z = 0.0

            path_msg.poses.append(pose)
            
        self.pub_path.publish(path_msg)

    def timer_cb(self):
        current_time = self.get_clock().now().nanoseconds / 1e9

        if current_time <= 0.0 or not self.robot_ready:
            return

        if self.t0 is None:
            self.t0 = current_time
            self.get_logger().info("Robot detected. Starting trajectory.")

        t = current_time - self.t0

        # After one loop, stop the trajectory
        if t >= self.t_max:
            t = self.t_max
            stop_flag = 1.0
        else:
            stop_flag = 0.0
        
        # Trajectory
        x_d, y_d = self.get_x(t), self.get_y(t)
        x_d_dot, y_d_dot = self.get_dx(t), self.get_dy(t)
        x_d_ddot, y_d_ddot = self.get_ddx(t), self.get_ddy(t)

        msg = Float64MultiArray()
        msg.data = [x_d, y_d, x_d_dot, y_d_dot, x_d_ddot, y_d_ddot, stop_flag]
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()