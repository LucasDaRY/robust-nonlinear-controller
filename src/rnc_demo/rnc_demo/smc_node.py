#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class SMCNode(Node):
    def __init__(self):
        super().__init__('smc_node')
        
        # Déclaration des paramètres configurables via le fichier YAML
        self.declare_parameter('lambda_p', 1.0)
        self.declare_parameter('lambda_i', 0.5)
        self.declare_parameter('lambda_d', 0.000001)
        self.declare_parameter('k', 0.02)
        self.declare_parameter('a', 0.05)
        self.declare_parameter('epsilon', 0.05)
        self.declare_parameter('v_max', 0.15)
        self.declare_parameter('w_max', 1.0)

        # Abonnements et Éditeur
        self.sub_pose = self.create_subscription(Pose2D, '/robot_pose', self.pose_cb, 10)
        self.sub_traj = self.create_subscription(Float64MultiArray, '/desired_trajectory', self.traj_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # State variables
        self.h = np.zeros((2, 1))
        self.psi = 0.0
        self.hd = np.zeros((2, 1))
        self.hd_dot = np.zeros((2, 1))
        self.hd_ddot = np.zeros((2, 1))

        self.is_finished = False

        # Mémoires pour intégration et dérivation
        self.err_int = np.zeros((2, 1))
        self.err_prev = np.zeros((2, 1))
        self.err_dot_prev = np.zeros((2, 1))

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.control_loop) # Boucle à 20 Hz

    def pose_cb(self, msg):
        self.h[0, 0] = msg.x
        self.h[1, 0] = msg.y
        self.psi = msg.theta

    def traj_cb(self, msg):
        self.hd[0, 0] = msg.data[0]
        self.hd[1, 0] = msg.data[1]
        self.hd_dot[0, 0] = msg.data[2]
        self.hd_dot[1, 0] = msg.data[3]
        self.hd_ddot[0, 0] = msg.data[4]
        self.hd_ddot[1, 0] = msg.data[5]
        self.is_finished = (msg.data[6] == 1.0)

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        # Stop after first lap
        if self.is_finished:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_cmd.publish(twist)
            return

        # Récupération dynamique des paramètres déclarés dans le YAML
        lambda_p = self.get_parameter('lambda_p').value
        lambda_i = self.get_parameter('lambda_i').value
        lambda_d = self.get_parameter('lambda_d').value
        k = self.get_parameter('k').value
        a = self.get_parameter('a').value
        epsilon = self.get_parameter('epsilon').value
        v_max = self.get_parameter('v_max').value
        w_max = self.get_parameter('w_max').value

        # Calculs des erreurs de suivi
        err = self.hd - self.h
        err_dot = (err - self.err_prev) / dt
        err_ddot = (err_dot - self.err_dot_prev) / dt
        self.err_int += err * dt

        # Surface glissante s = lambda_p * err + lambda_i * int(err) + lambda_d * err_dot
        s = lambda_p * err + lambda_i * self.err_int + lambda_d * err_dot

        # Matrice Jacobienne inverse
        J_inv = np.array([
            [np.cos(self.psi), np.sin(self.psi)],
            [-np.sin(self.psi) / a, np.cos(self.psi) / a]
        ])

        # Partie continue ua = J^-1 * [ hd_dot + (lambda_i/lambda_p)*err + (lambda_d/lambda_p)*err_ddot ]
        term_ua = self.hd_dot + (lambda_i / lambda_p) * err + (lambda_d / lambda_p) * err_ddot
        ua = J_inv @ term_ua

        # Partie discontinue adoucie ud = J^-1 * k * tanh(s / epsilon)
        ud = J_inv @ (k * np.tanh(s / epsilon))

        # Commande totale U = ua + ud
        U = ua + ud

        # Mémorisation pour le pas suivant
        self.err_prev = err
        self.err_dot_prev = err_dot

        # Saturation aux limites de vitesse configurées
        twist = Twist()
        twist.linear.x = float(np.clip(U[0, 0], -v_max, v_max))
        twist.angular.z = float(np.clip(U[1, 0], -w_max, w_max))
        self.pub_cmd.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SMCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()