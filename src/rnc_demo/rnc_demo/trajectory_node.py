#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
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

        # Gazebo has no path display. Spawn a visual built from get_x/get_y so an
        # edit of the equations above is what the world shows on the next run.
        self._gazebo_model = 'figure8_trajectory'
        self._gazebo_done = False
        self._gazebo_inflight = False
        self._gazebo_cleared = False
        self._gazebo_retried = False
        self._gazebo_attempts = 0
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.timer_gazebo = self.create_timer(1.0, self._draw_in_gazebo)

    def _curve_points(self, samples):
        points = []
        for i in range(samples + 1):
            t = self.t_max * i / samples
            points.append((float(self.get_x(t)), float(self.get_y(t))))
        return points

    def _trajectory_sdf(self):
        visuals = []
        radius = 0.012
        z = 0.02
        segment = 0
        points = self._curve_points(240)
        for (x1, y1), (x2, y2) in zip(points, points[1:]):
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length < 1e-4:
                continue
            yaw = math.atan2(dy, dx)
            visuals.append(
                '<visual name="seg_{segment}">'
                '<pose>{x:.5f} {y:.5f} {z:.5f} 0 1.5707963 {yaw:.6f}</pose>'
                '<geometry><cylinder><radius>{radius}</radius>'
                '<length>{length:.5f}</length></cylinder></geometry>'
                '<material>'
                '<ambient>0.05 0.7 0.15 1</ambient>'
                '<diffuse>0.1 0.9 0.25 1</diffuse>'
                '<emissive>0.02 0.35 0.08 1</emissive>'
                '</material>'
                '</visual>'.format(
                    segment=segment,
                    x=(x1 + x2) / 2.0,
                    y=(y1 + y2) / 2.0,
                    z=z,
                    yaw=yaw,
                    radius=radius,
                    length=length,
                )
            )
            segment += 1

        return (
            '<?xml version="1.0"?>'
            '<sdf version="1.6"><model name="{name}">'
            '<static>true</static><link name="path">'
            '<inertial><mass>0.001</mass><inertia>'
            '<ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz>'
            '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>'
            '</inertia></inertial>'
            '{visuals}'
            '</link></model></sdf>'
        ).format(name=self._gazebo_model, visuals=''.join(visuals))

    def _draw_in_gazebo(self):
        if self._gazebo_done or self._gazebo_inflight:
            return
        self._gazebo_attempts += 1
        if not self.spawn_client.service_is_ready():
            if self._gazebo_attempts >= 15:
                self.get_logger().info(
                    'Gazebo /spawn_entity is not available. The trajectory stays on /desired_path.'
                )
                self._gazebo_done = True
                self.timer_gazebo.cancel()
            return

        self._gazebo_inflight = True
        if self.delete_client.service_is_ready() and not self._gazebo_cleared:
            request = DeleteEntity.Request()
            request.name = self._gazebo_model
            self.delete_client.call_async(request).add_done_callback(self._after_delete)
            return
        self._send_spawn()

    def _after_delete(self, future):
        self._gazebo_cleared = True
        self._gazebo_inflight = False
        try:
            future.result()
        except Exception:
            pass
        self._send_spawn()

    def _send_spawn(self):
        self._gazebo_inflight = True
        request = SpawnEntity.Request()
        request.name = self._gazebo_model
        request.xml = self._trajectory_sdf()
        request.reference_frame = 'world'
        request.initial_pose.orientation.w = 1.0
        self.spawn_client.call_async(request).add_done_callback(self._after_spawn)

    def _after_spawn(self, future):
        self._gazebo_inflight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'Gazebo did not draw the trajectory: {exc}')
            self._gazebo_done = True
            self.timer_gazebo.cancel()
            return
        if response.success:
            self.get_logger().info('Drew the current trajectory equations in Gazebo.')
            self._gazebo_done = True
            self.timer_gazebo.cancel()
            return
        if (not self._gazebo_retried) and 'exist' in response.status_message.lower():
            self._gazebo_retried = True
            self._gazebo_cleared = False
            return
        self.get_logger().warning(
            f'Gazebo rejected the trajectory: {response.status_message}'
        )
        self._gazebo_done = True
        self.timer_gazebo.cancel()

    def pose_cb(self, msg):
        self.robot_ready = True

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(1000):
            t_sim = i * 0.5 # second per point
            # Automatic stop
            if t_sim > self.t_max:
                break

            pose = PoseStamped()
            pose.header.frame_id = "map"
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