#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        # Subscription to Gazebo/TurtleBot odometry topic
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        # Publish simplified position
        self.pub = self.create_publisher(Pose2D, '/robot_pose', 10)

    def odom_cb(self, msg):
        pose = Pose2D()
        pose.x = msg.pose.pose.position.x
        pose.y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw angle
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        pose.theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()