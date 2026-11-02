#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        
        # The publisher remains identical; the SMC node is unaware of the backend change
        self.pub = self.create_publisher(Pose2D, '/robot_pose', 10)
        
        # Tools to listen to the Transform Tree (TF2)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Query the TF tree at 30 Hz to ensure a smooth signal for the SMC
        self.timer = self.create_timer(1.0 / 30.0, self.timer_cb)

    def timer_cb(self):
        try:
            # Lookup the transform between the absolute map and the robot
            t = self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                rclpy.time.Time())
            
            pose = Pose2D()
            pose.x = t.transform.translation.x
            pose.y = t.transform.translation.y
            
            # Convert quaternion to Yaw angle
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            pose.theta = math.atan2(siny_cosp, cosy_cosp)
            
            self.pub.publish(pose)
            
        except TransformException:
            # Expected behavior during the first few seconds before SLAM is ready
            pass

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()