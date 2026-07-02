from functools import partial

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('danger_distance', 20.0)
        self.declare_parameter('safety_buffer_radius', 2.0)
        self.declare_parameter('obstacle_trigger_margin', 8.0)
        self.declare_parameter('dynamic_obstacle_topics', ['/vessel_e/odom', '/vessel_f/odom'])
        self.declare_parameter('dynamic_obstacle_names', ['vessel_e', 'vessel_f'])
        self.declare_parameter('dynamic_obstacle_radii', [3.0, 3.0])
        self.declare_parameter('vessel_safety_margin', 2.0)
        self.declare_parameter('usv_collision_radius', 2.0)
        self.declare_parameter('dynamic_obstacle_timeout_sec', 0.5)
        self.declare_parameter('perception_publish_rate', 10.0)
        self.declare_parameter('offset', [0.0, 0.0, 0.0])
        self.declare_parameter('obstacle_bearing_limit_deg', 100.0)
        self.declare_parameter('obstacle_cpa_horizon_sec', 35.0)
        self.declare_parameter('avoidance_entry_count', 2)
        self.declare_parameter('avoidance_exit_count', 5)

        self.danger_distance = float(self.get_parameter('danger_distance').value)
        self.safety_buffer_radius = float(self.get_parameter('safety_buffer_radius').value)
        self.obstacle_trigger_margin = float(self.get_parameter('obstacle_trigger_margin').value)
        self.dynamic_obstacle_topics = list(self.get_parameter('dynamic_obstacle_topics').value)
        self.dynamic_obstacle_names = list(self.get_parameter('dynamic_obstacle_names').value)
        self.dynamic_obstacle_radii = list(self.get_parameter('dynamic_obstacle_radii').value)
        self.vessel_safety_margin = float(self.get_parameter('vessel_safety_margin').value)
        self.usv_collision_radius = float(self.get_parameter('usv_collision_radius').value)
        self.dynamic_obstacle_timeout_sec = float(self.get_parameter('dynamic_obstacle_timeout_sec').value)
        self.perception_publish_rate = max(1.0, float(self.get_parameter('perception_publish_rate').value))
        self.obstacle_bearing_limit = np.deg2rad(float(self.get_parameter('obstacle_bearing_limit_deg').value))
        self.obstacle_cpa_horizon_sec = float(self.get_parameter('obstacle_cpa_horizon_sec').value)
        self.avoidance_entry_count = max(1, int(self.get_parameter('avoidance_entry_count').value))
        self.avoidance_exit_count = max(1, int(self.get_parameter('avoidance_exit_count').value))
        initial_offset = list(self.get_parameter('offset').value)
        if len(initial_offset) >= 2:
            self.current_pos = np.array([float(initial_offset[0]), float(initial_offset[1])], dtype=float)
        else:
            self.current_pos = None

        self.ns = self.get_namespace()
        if self.ns == '/':
            self.ns = '/usv_1'
        self.agent_name = self.ns.strip('/')

        self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.mode_pub = self.create_publisher(String, f'/{self.agent_name}/planner_mode', 10)
        self.obs_pub = self.create_publisher(Point, f'/{self.agent_name}/detected_obstacle', 10)
        self.current_mode = 'FORMATION'
        self.obstacle_tracks = {}
        self.last_debug_log_time = 0.0
        self.last_pos = None
        self.last_pos_time = None
        self.current_vel = np.zeros(2, dtype=float)
        self.avoidance_seen_count = 0
        self.avoidance_clear_count = 0

        obstacle_count = min(
            len(self.dynamic_obstacle_topics),
            len(self.dynamic_obstacle_names),
            len(self.dynamic_obstacle_radii)
        )
        for i in range(obstacle_count):
            name = str(self.dynamic_obstacle_names[i])
            topic = str(self.dynamic_obstacle_topics[i])
            radius = float(self.dynamic_obstacle_radii[i])
            self.obstacle_tracks[name] = {
                'position': None,
                'velocity': np.zeros(2, dtype=float),
                'radius': radius,
                'stamp': None,
            }
            self.create_subscription(
                Odometry,
                topic,
                partial(self.dynamic_obstacle_callback, obstacle_name=name),
                10
            )

        self.create_timer(1.0 / self.perception_publish_rate, self.mode_timer_callback)
        self.get_logger().info(
            f'Perception Node started for {self.agent_name}, '
            f'tracking obstacles={list(self.obstacle_tracks.keys())}'
        )

    def odom_callback(self, msg):
        now_t = self.get_clock().now().nanoseconds * 1e-9
        p = msg.pose.pose.position
        new_pos = np.array([p.x, p.y], dtype=float)
        if self.current_pos is not None:
            dt = now_t - self.last_pos_time if self.last_pos_time is not None else 0.0
            if dt > 1e-6:
                self.current_vel = (new_pos - self.current_pos) / dt
        self.last_pos = self.current_pos
        self.last_pos_time = now_t
        self.current_pos = new_pos

    def dynamic_obstacle_callback(self, msg, obstacle_name):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        track = self.obstacle_tracks[obstacle_name]
        track['position'] = np.array([p.x, p.y], dtype=float)
        track['velocity'] = np.array([v.x, v.y], dtype=float)
        track['stamp'] = self.get_clock().now().nanoseconds * 1e-9

    def is_obstacle_relevant(self, delta, clearance, radius, obstacle_velocity):
        speed = float(np.linalg.norm(self.current_vel))
        if speed < 0.1:
            return clearance <= self.obstacle_trigger_margin + radius

        heading = self.current_vel / speed
        dist = float(np.linalg.norm(delta))
        if dist < 1e-6:
            return True

        bearing = abs(np.arctan2(heading[0] * delta[1] - heading[1] * delta[0], float(np.dot(heading, delta))))
        if bearing > self.obstacle_bearing_limit and clearance > self.obstacle_trigger_margin + radius:
            return False

        rel_vel = self.current_vel - obstacle_velocity
        rel_speed_sq = float(np.dot(rel_vel, rel_vel))
        if rel_speed_sq < 1e-6:
            return clearance <= self.obstacle_trigger_margin

        t_cpa = float(np.clip(np.dot(delta, rel_vel) / rel_speed_sq, 0.0, self.obstacle_cpa_horizon_sec))
        cpa_dist = float(np.linalg.norm(delta - rel_vel * t_cpa))
        return cpa_dist <= radius + self.obstacle_trigger_margin

    def mode_timer_callback(self):
        now_t = self.get_clock().now().nanoseconds * 1e-9
        if self.current_pos is None:
            return

        visible_count = 0
        min_clearance = None
        range_limit = self.danger_distance + self.obstacle_trigger_margin

        for name, track in self.obstacle_tracks.items():
            if track['position'] is None or track['stamp'] is None:
                continue
            if now_t - track['stamp'] > self.dynamic_obstacle_timeout_sec:
                continue

            delta = track['position'] - self.current_pos
            inflated_radius = track['radius'] + self.usv_collision_radius + self.vessel_safety_margin
            clearance = float(np.linalg.norm(delta) - inflated_radius)
            if min_clearance is None or clearance < min_clearance:
                min_clearance = clearance

            if clearance > range_limit:
                continue
            if not self.is_obstacle_relevant(delta, clearance, inflated_radius, track['velocity']):
                continue

            obs_msg = Point()
            obs_msg.x = float(track['position'][0])
            obs_msg.y = float(track['position'][1])
            obs_msg.z = float(inflated_radius)
            self.obs_pub.publish(obs_msg)
            visible_count += 1

        if visible_count > 0:
            self.avoidance_seen_count += 1
            self.avoidance_clear_count = 0
        else:
            self.avoidance_clear_count += 1
            self.avoidance_seen_count = 0

        if self.current_mode == 'FORMATION':
            new_mode = 'AVOIDANCE' if self.avoidance_seen_count >= self.avoidance_entry_count else 'FORMATION'
        else:
            new_mode = 'FORMATION' if self.avoidance_clear_count >= self.avoidance_exit_count else 'AVOIDANCE'
        if new_mode != self.current_mode:
            self.get_logger().info(
                f'{self.agent_name} mode {self.current_mode} -> {new_mode}, '
                f'visible_dynamic_obstacles={visible_count}, min_clearance={min_clearance}'
            )
            self.current_mode = new_mode

        if now_t - self.last_debug_log_time > 1.0:
            self.last_debug_log_time = now_t
            self.get_logger().info(
                f'{self.agent_name} perception: mode={self.current_mode}, '
                f'visible_dynamic_obstacles={visible_count}, min_clearance={min_clearance}'
            )

        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_pub.publish(mode_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
