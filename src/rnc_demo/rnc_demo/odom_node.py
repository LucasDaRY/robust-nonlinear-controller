#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from rnc_demo.imu_odom import PlanarDisturbanceOdom, wrap


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _quat_to_rot(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-8:
        return None
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def _rotate(rot, vector):
    return (
        rot[0][0] * vector[0] + rot[0][1] * vector[1] + rot[0][2] * vector[2],
        rot[1][0] * vector[0] + rot[1][1] * vector[1] + rot[1][2] * vector[2],
        rot[2][0] * vector[0] + rot[2][1] * vector[1] + rot[2][2] * vector[2],
    )


def _yaw_rotation(yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        (c, -s, 0.0),
        (s, c, 0.0),
        (0.0, 0.0, 1.0),
    )


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        self.declare_parameter('accel_threshold', 0.45)
        self.declare_parameter('yaw_rate_threshold', 0.4)
        self.declare_parameter('settle_time', 0.4)
        self.declare_parameter('max_event_time', 4.0)
        self.declare_parameter('debounce_time', 0.05)
        self.declare_parameter('gravity', 9.81)

        self.fuse = PlanarDisturbanceOdom(
            accel_threshold=self.get_parameter('accel_threshold').value,
            yaw_rate_threshold=self.get_parameter('yaw_rate_threshold').value,
            settle_time=self.get_parameter('settle_time').value,
            max_event_time=self.get_parameter('max_event_time').value,
            debounce_time=self.get_parameter('debounce_time').value,
        )
        self._gravity = self.get_parameter('gravity').value
        self._reported = False
        self._last_mode = 'tracking'
        self._last_path_time = None

        self.pub = self.create_publisher(Pose2D, '/robot_pose', 10)
        self.pub_disturbance = self.create_publisher(Bool, '/disturbance_active', 10)
        self.pub_actual_path = self.create_publisher(Path, '/actual_path', 10)
        self._actual_path_msg = Path()
        self._actual_path_msg.header.frame_id = 'odom'

        # Wheel odometry is the pose. The IMU is used only while the wheels slip.
        self.create_subscription(Odometry, '/odom', self._wheel_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self._imu_cb, qos_profile_sensor_data)

    def _wheel_cb(self, msg):
        if not self._reported:
            self._reported = True
            self.get_logger().info(
                'Pose source is wheel /odom. IMU integration runs only during a detected shove.'
            )
        pose = msg.pose.pose
        stamp = self._stamp(msg.header.stamp)
        self.fuse.wheel(
            pose.position.x,
            pose.position.y,
            _yaw_from_quat(pose.orientation),
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
            stamp,
        )
        self._publish(msg.header.frame_id or 'odom', msg.header.stamp)

    def _imu_cb(self, msg):
        stamp = self._stamp(msg.header.stamp)
        rotation = _quat_to_rot(msg.orientation)
        if rotation is None:
            rotation = _yaw_rotation(self.fuse.yaw)
        accel = _rotate(
            rotation,
            (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
        )
        gyro = _rotate(
            rotation,
            (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
        )
        # Specific force with gravity removed. The estimator keeps only x and y.
        self.fuse.imu(
            (accel[0], accel[1], accel[2] - self._gravity),
            gyro[2],
            stamp,
        )
        if self.fuse.forced_exit and self.fuse.mode == 'tracking' and self._last_mode != 'tracking':
            self.get_logger().warning('IMU correction hit its time limit. Back on wheel odometry.')
            self.fuse.forced_exit = False
        self._publish('odom', msg.header.stamp)

    def _publish(self, frame_id, stamp):
        mode = self.fuse.mode
        if mode != self._last_mode:
            if mode == 'disturbance':
                self.get_logger().info('Shove detected. Wheels are ignored until it settles.')
            elif mode == 'tracking' and self._last_mode != 'tracking':
                self.get_logger().info('Shove settled. Pose is back on wheel odometry.')
            self._last_mode = mode

        pose = Pose2D()
        pose.x = self.fuse.x
        pose.y = self.fuse.y
        pose.theta = wrap(self.fuse.yaw)
        self.pub.publish(pose)

        flag = Bool()
        flag.data = self.fuse.disturbance
        self.pub_disturbance.publish(flag)

        now = self._stamp(stamp)
        if self._last_path_time is not None and now - self._last_path_time < 0.1:
            return
        self._last_path_time = now
        stamped = PoseStamped()
        stamped.header.frame_id = frame_id
        stamped.header.stamp = stamp
        stamped.pose.position.x = self.fuse.x
        stamped.pose.position.y = self.fuse.y
        stamped.pose.orientation.z = math.sin(pose.theta / 2.0)
        stamped.pose.orientation.w = math.cos(pose.theta / 2.0)
        self._actual_path_msg.header.frame_id = frame_id
        self._actual_path_msg.header.stamp = stamp
        self._actual_path_msg.poses.append(stamped)
        self.pub_actual_path.publish(self._actual_path_msg)

    def _stamp(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
