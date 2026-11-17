#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        
        self.pub = self.create_publisher(Pose2D, '/robot_pose', 10)

        # Publish the actual robot path for RViz overlay
        self.pub_actual_path = self.create_publisher(Path, '/actual_path', 10)
        self._actual_path_msg = Path()
        self._actual_path_msg.header.frame_id = 'map'
        
        # Tools to listen to the Transform Tree (TF2)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.initial_pose = None
        
        # Query the TF tree at 30 Hz to ensure a smooth signal for the SMC
        self.timer = self.create_timer(1.0 / 30.0, self.timer_cb)

    def timer_cb(self):
        try:
            # Lookup the transform published by Cartographer: map → base_footprint
            # (Cartographer publishes map → odom; TF2 resolves the full chain
            # map → odom → base_footprint automatically via lookup_transform)
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time())
            
            x_map = t.transform.translation.x
            y_map = t.transform.translation.y
            
            # Convert quaternion to Yaw angle
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta_map = math.atan2(siny_cosp, cosy_cosp)

            # Tare initial pose so the controller always starts at (0, 0, 0)
            if self.initial_pose is None:
                self.initial_pose = (x_map, y_map, theta_map)
                return

            dx = x_map - self.initial_pose[0]
            dy = y_map - self.initial_pose[1]
            theta0 = self.initial_pose[2]

            pose = Pose2D()
            pose.x = dx * math.cos(theta0) + dy * math.sin(theta0)
            pose.y = -dx * math.sin(theta0) + dy * math.cos(theta0)
            pose.theta = (theta_map - theta0 + math.pi) % (2.0 * math.pi) - math.pi
            
            self.pub.publish(pose)

            # Accumulate actual path for RViz
            stamped = PoseStamped()
            stamped.header.frame_id = 'map'
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.pose.position.x = t.transform.translation.x
            stamped.pose.position.y = t.transform.translation.y
            stamped.pose.position.z = 0.0
            stamped.pose.orientation = t.transform.rotation
            self._actual_path_msg.poses.append(stamped)
            self._actual_path_msg.header.stamp = stamped.header.stamp
            self.pub_actual_path.publish(self._actual_path_msg)
            
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