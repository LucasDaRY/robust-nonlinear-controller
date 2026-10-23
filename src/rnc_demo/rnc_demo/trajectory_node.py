#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import time

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        
        # Pour le contrôleur SMC
        self.pub_cmd = self.create_publisher(Float64MultiArray, '/desired_trajectory', 10)
        
        # Pour dessiner la courbe dans RViz
        self.pub_path = self.create_publisher(Path, '/desired_path', 10)
        
        self.timer_cmd = self.create_timer(0.05, self.timer_cb) # 20 Hz pour le contrôle
        self.timer_path = self.create_timer(1.0, self.publish_path) # 1 Hz pour le dessin statique
        
        self.t0 = time.time()

    # J'ai isolé l'équation de la position ici pour que vous 
    # puissiez la changer facilement à un seul endroit !
    def get_desired_pose(self, t):
        x = 0.5 * math.cos(t / 10.0) + 0.5
        y = 0.5 * math.sin(t / 10.0) + 0.5
        return x, y

    def publish_path(self):
        """ Échantillonne l'équation paramétrique pour dessiner le futur de la trajectoire """
        path_msg = Path()
        path_msg.header.frame_id = "odom" # Le repère fixe du monde
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # On va calculer la courbe sur 100 secondes (t_sim de 0 à 100)
        # pour dessiner la ligne complète sur le sol
        for i in range(200):
            t_sim = i * 0.5  # Un point toutes les 0.5 secondes
            x, y = self.get_desired_pose(t_sim)
            
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0 # Posé sur le sol
            
            path_msg.poses.append(pose)
            
        self.pub_path.publish(path_msg)

    def timer_cb(self):
        t = time.time() - self.t0
        
        # Position désirée à l'instant t
        x_d, y_d = self.get_desired_pose(t)
        
        # TODO: Mettre à jour les dérivées ici 
        x_d_dot = -0.05 * math.sin(t / 10.0)
        y_d_dot = 0.05 * math.cos(t / 10.0)
        
        x_d_ddot = -0.005 * math.cos(t / 10.0)
        y_d_ddot = -0.005 * math.sin(t / 10.0)

        msg = Float64MultiArray()
        msg.data = [x_d, y_d, x_d_dot, y_d_dot, x_d_ddot, y_d_ddot]
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()