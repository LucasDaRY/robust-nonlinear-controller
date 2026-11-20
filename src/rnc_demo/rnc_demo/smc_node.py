#!/usr/bin/env python3
import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64MultiArray


class SMCNode(Node):
    def __init__(self):
        super().__init__('smc_node')

        # Paramètres configurables via YAML
        self.declare_parameter('lambda_p', 1.2)
        self.declare_parameter('lambda_i', 0.5)
        self.declare_parameter('lambda_d', 0.0)
        self.declare_parameter('k', 0.03)
        self.declare_parameter('a', 0.12)
        self.declare_parameter('epsilon', 0.08)
        self.declare_parameter('v_max', 0.18)
        self.declare_parameter('w_max', 0.6)
        self.declare_parameter('filter_alpha', 0.25)

        # Souscriptions et publications
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_cb, 10
        )
        self.sub_traj = self.create_subscription(
            Float64MultiArray, '/desired_trajectory', self.traj_cb, 10
        )
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Chemin réel pour RViz2
        self.pub_actual_path = self.create_publisher(Path, '/actual_path', 10)
        self._actual_path_msg = Path()
        self._actual_path_msg.header.frame_id = 'map'

        # Variables d'état filtrées
        self.x = 0.0
        self.y = 0.0
        self.cos_psi = 1.0
        self.sin_psi = 0.0
        self.psi = 0.0
        self.h = np.zeros((2, 1))
        self.hd = np.zeros((2, 1))
        self.hd_dot = np.zeros((2, 1))
        self.hd_ddot = np.zeros((2, 1))

        self.has_odom = False
        self.has_traj = False
        self.is_finished = False
        self.first_control_step = True

        # Mémoire pour intégration et dérivation
        self.err_int = np.zeros((2, 1))
        self.err_prev = np.zeros((2, 1))
        self.err_dot_prev = np.zeros((2, 1))

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # Sauvegarde CSV
        self.csv_path = os.path.expanduser('trajectory_data.csv')
        self.csv_file = open(self.csv_path, 'w')
        self.csv_file.write("x_des,y_des,x_real,y_real,x_center,y_center\n")

    def odom_cb(self, msg: Odometry):
        a = self.get_parameter('a').value
        alpha = self.get_parameter('filter_alpha').value

        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_psi = math.atan2(siny_cosp, cosy_cosp)

        if not self.has_odom:
            self.x = raw_x
            self.y = raw_y
            self.cos_psi = math.cos(raw_psi)
            self.sin_psi = math.sin(raw_psi)
            self.has_odom = True
        else:
            self.x = alpha * raw_x + (1.0 - alpha) * self.x
            self.y = alpha * raw_y + (1.0 - alpha) * self.y
            self.cos_psi = alpha * math.cos(raw_psi) + (1.0 - alpha) * self.cos_psi
            self.sin_psi = alpha * math.sin(raw_psi) + (1.0 - alpha) * self.sin_psi

        self.psi = math.atan2(self.sin_psi, self.cos_psi)

        # Mise à jour du point de contrôle déporté h
        self.h[0, 0] = self.x + a * math.cos(self.psi)
        self.h[1, 0] = self.y + a * math.sin(self.psi)

        # Publication du chemin filtré pour RViz2
        stamped = PoseStamped()
        stamped.header.frame_id = 'map'
        stamped.header.stamp = msg.header.stamp
        stamped.pose.position.x = self.x
        stamped.pose.position.y = self.y
        stamped.pose.orientation.z = math.sin(0.5 * self.psi)
        stamped.pose.orientation.w = math.cos(0.5 * self.psi)
        self._actual_path_msg.poses.append(stamped)
        self._actual_path_msg.header.stamp = stamped.header.stamp
        self.pub_actual_path.publish(self._actual_path_msg)

    def traj_cb(self, msg: Float64MultiArray):
        self.hd[0, 0] = msg.data[0]
        self.hd[1, 0] = msg.data[1]
        self.hd_dot[0, 0] = msg.data[2]
        self.hd_dot[1, 0] = msg.data[3]
        self.hd_ddot[0, 0] = msg.data[4]
        self.hd_ddot[1, 0] = msg.data[5]
        self.is_finished = (msg.data[6] == 1.0)
        self.has_traj = True

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or not (self.has_odom and self.has_traj):
            return
        self.last_time = now

        if not self.csv_file.closed and not self.is_finished:
            self.csv_file.write(
                f"{self.hd[0,0]},{self.hd[1,0]},{self.h[0,0]},{self.h[1,0]},{self.x},{self.y}\n"
            )

        if self.is_finished:
            self.pub_cmd.publish(Twist())
            return

        lambda_p = self.get_parameter('lambda_p').value
        lambda_i = self.get_parameter('lambda_i').value
        lambda_d = self.get_parameter('lambda_d').value
        k = self.get_parameter('k').value
        a = self.get_parameter('a').value
        epsilon = self.get_parameter('epsilon').value
        v_max = self.get_parameter('v_max').value
        w_max = self.get_parameter('w_max').value

        err = self.hd - self.h
        if self.first_control_step:
            self.err_prev = err.copy()
            self.first_control_step = False

        err_dot = (err - self.err_prev) / dt
        err_ddot = (err_dot - self.err_dot_prev) / dt
        self.err_int = np.clip(self.err_int + err * dt, -0.4, 0.4)

        s = lambda_p * err + lambda_i * self.err_int + lambda_d * err_dot

        J_inv = np.array([
            [math.cos(self.psi), math.sin(self.psi)],
            [-math.sin(self.psi) / a, math.cos(self.psi) / a]
        ])

        term_ua = self.hd_dot + (lambda_i / lambda_p) * err + (lambda_d / lambda_p) * err_ddot
        ua = J_inv @ term_ua
        ud = J_inv @ (k * np.tanh(s / epsilon))
        U = ua + ud

        self.err_prev = err
        self.err_dot_prev = err_dot

        twist = Twist()
        twist.linear.x = float(np.clip(U[0, 0], -v_max, v_max))
        twist.angular.z = float(np.clip(U[1, 0], -w_max, w_max))
        self.pub_cmd.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SMCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.csv_file.closed:
            node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()