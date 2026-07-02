import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from topology_interfaces.msg import TopologicalPathArray, Decision
from std_msgs.msg import String
import numpy as np
import time
import json
import urllib.request
from geometry_msgs.msg import PoseStamped, Point
from collections import deque
from trajectory_smoother.path_costs import path_length as compute_path_length
from trajectory_smoother.path_costs import sample_path_xy as compute_sample_path_xy
from trajectory_smoother.smoother_params import declare_smoother_parameters
from trajectory_smoother.smoother_params import get_colregs_angles, get_float, get_int, get_list, get_static_obstacles

class SmootherNode(Node):
    def __init__(self,smoother_node):
        super().__init__(smoother_node)
        
        self.declare_parameter("agent_id", 1)
        self.declare_parameter("num_usvs", 4)
        self.my_id = self.get_parameter("agent_id").value
        self.num_usvs = self.get_parameter("num_usvs").value
        self.agent_name = f"usv_{self.my_id}"
        self._debug_url, self._debug_session = self._load_debug_env()
        declare_smoother_parameters(self, self.my_id)

        self.comm_radius = get_float(self, "comm_radius")
        self.all_possible_neighbors = [i for i in range(1, self.num_usvs + 1) if i != self.my_id]
        configured_neighbors = [int(n) for n in get_list(self, f"usv_{self.my_id}_neighbors")]
        self.active_neighbors = [n for n in configured_neighbors if n in self.all_possible_neighbors]
        self.offsets = self.get_parameter(f"usv_{self.my_id}_offset").value
        self.global_target = self.get_parameter("target").value
        
        self.goal_subscriber = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        
        self.k_form = get_float(self, "formation_coupling_gain")
        self.comm_delay_sec = get_float(self, "comm_delay_sec")
        self.avoidance_decision_hold_sec = get_float(self, "avoidance_decision_hold_sec")
        self.local_avoid_trigger_margin = get_float(self, "local_avoid_trigger_margin")
        self.safety_buffer_radius = get_float(self, "safety_buffer_radius")
        self.static_obstacle_safety_margin = get_float(self, "static_obstacle_safety_margin")
        self.shared_obstacle_distance_margin = get_float(self, "shared_obstacle_distance_margin")
        self.shared_obstacle_front_margin = get_float(self, "shared_obstacle_front_margin")
        self.shared_obstacle_z_margin = get_float(self, "shared_obstacle_z_margin")
        self.obstacle_priority_distance = get_float(self, "obstacle_priority_distance")
        self.obstacle_critical_clearance = get_float(self, "obstacle_critical_clearance")
        self.neighbor_progress_margin = get_float(self, "neighbor_progress_margin")
        self.neighbor_discussion_distance = get_float(self, "neighbor_discussion_distance")
        self.emergency_lock_margin = get_float(self, "emergency_lock_margin")
        self.stop_radius = get_float(self, "stop_radius")
        self.exit_clearance_margin = get_float(self, "exit_clearance_margin")
        self.exit_hold_count_required = max(1, get_int(self, "exit_hold_count"))
        self.collision_penalty = get_float(self, "collision_penalty")
        self.colregs_penalty = get_float(self, "colregs_penalty")
        self.consistency_penalty = get_float(self, "consistency_penalty")
        self.path_switch_penalty = get_float(self, "path_switch_penalty")
        self.mode_switch_hysteresis = get_float(self, "mode_switch_hysteresis")
        self.single_path_yield_penalty = get_float(self, "single_path_yield_penalty")
        self.soft_conflict_gain = get_float(self, "soft_conflict_gain")
        self.path_sample_count = get_int(self, "path_sample_count")
        angles = get_colregs_angles(self)
        self.head_on_heading_tol = angles["head_on_heading_tol"]
        self.head_on_bearing_tol = angles["head_on_bearing_tol"]
        self.overtaking_heading_tol = angles["overtaking_heading_tol"]
        self.overtaking_bearing = angles["overtaking_bearing"]
        self.crossing_progress_margin = get_float(self, "crossing_progress_margin")
        self.decision_settle_sec = get_float(self, "decision_settle_sec")
        self.formation_path_points = get_int(self, "formation_path_points")
        self.route_lock_repeat_count = get_int(self, "route_lock_repeat_count")
        self.candidate_freeze_sec = get_float(self, "candidate_freeze_sec")
        self.dynamic_obstacle_topics = get_list(self, "dynamic_obstacle_topics")
        self.dynamic_obstacle_names = get_list(self, "dynamic_obstacle_names")
        self.dynamic_obstacle_radii = get_list(self, "dynamic_obstacle_radii")
        self.static_obstacles = get_static_obstacles(self)
        self.usv_perception_radius = get_float(self, "usv_perception_radius")
        self.usv_collision_radius = get_float(self, "usv_collision_radius")
        self.vessel_safety_margin = get_float(self, "vessel_safety_margin")
        self.spatiotemporal_horizon_sec = get_float(self, "spatiotemporal_horizon_sec")
        self.spatiotemporal_soft_margin = get_float(self, "spatiotemporal_soft_margin")
        self.relative_closing_speed_threshold = get_float(self, "relative_closing_speed_threshold")
        self.min_pairwise_ttc_sec = get_float(self, "min_pairwise_ttc_sec")
        self.usv_non_neighbor_colregs_gain = get_float(self, "usv_non_neighbor_colregs_gain")

        self.get_logger().info(f"我是 USV {self.my_id}, 通信邻居={self.active_neighbors}, 偏置是 {self.offsets}, 全局目标是 {self.global_target}")
        self.get_logger().info(f"编队耦合增益 k_form={self.k_form:.3f}, 通信时延={self.comm_delay_sec:.2f}s")
        
        # 记录所有 USV 的编队偏置（至少包括自己和所有潜在邻居）
        self.all_offsets = {self.my_id: np.array(self.offsets, dtype=float)}
        for n_id in self.all_possible_neighbors:
            offset_param = f"usv_{n_id}_offset"
            if not self.has_parameter(offset_param):
                self.declare_parameter(offset_param, [0.0, 0.0, 0.0])
            self.all_offsets[n_id] = np.array(self.get_parameter(offset_param).value, dtype=float)
        
        # 保存邻居的决策： {neighbor_id: {'h_sig': [], 'path': Path()}}
        self.neighbor_decisions = {}
        self.neighbor_topo_sets = {}
        
        self.current_topo_paths = None
        self.nav_costs = []
        self.my_last_choice = None
        self.my_last_signature = None
        
        # 新增：博弈轮次控制
        self.game_round = 0
        self.MAX_GAME_ROUNDS = int(self.get_parameter("max_game_rounds").value)
        self.game_locked = False
        self.lock_timer = None
        self.choice_stability_count = 0
        self.last_candidate_change_time = -1.0
        self.last_discussion_neighbors = set()
        
        # 订阅自己的 PRM 生成的拓扑路径备选方案 (使用相对话题，因为节点在 namespace 内部)
        self.path_subscriber_=self.create_subscription(TopologicalPathArray, "topological_path", self.path_callback, 10)
        
        self.smooth_publisher = self.create_publisher(Path, "smooth_trajectory", 10)
        self.decision_publisher_ = self.create_publisher(Decision, "topology_decision", 10)
        
        self.requested_mode = "FORMATION"
        self.current_mode = "FORMATION"
        self.avoidance_lock_until = -1.0
        # 订阅由感知节点发出的模式切换指令
        self.mode_subscriber = self.create_subscription(String, f"/{self.agent_name}/planner_mode", self.mode_callback, 10)
        
        # 直接订阅障碍船 odom，不再经过 perception_node 转发
        self.obstacle_odom_subs = []
        self._obstacle_tracks = {}
        for i in range(min(len(self.dynamic_obstacle_topics), len(self.dynamic_obstacle_names))):
            name = self.dynamic_obstacle_names[i]
            topic = self.dynamic_obstacle_topics[i]
            self._obstacle_tracks[name] = {'pos': None, 'stamp': 0.0}
            sub = self.create_subscription(
                Odometry, topic,
                lambda msg, n=name: self._obs_odom_callback(n, msg), 10)
            self.obstacle_odom_subs.append(sub)
        self._obstacle_timeout = float(self.get_parameter("dynamic_obstacle_timeout_sec").value)
        self._obstacle_prune_timer = self.create_timer(0.5, self._prune_obstacle_tracks)
        
        self.my_position = None
        self.my_heading = None
        self.exit_hold_count = 0
        self.has_exited_avoidance_region = False
        self.sub_my_odom = self.create_subscription(Odometry, "odom", self.my_odom_callback, 10)
        
        # 订阅所有潜在邻居 odom，并维护历史队列（用于可选时延）
        self.neighbor_odom_subscribers = []
        self.neighbor_odom_history = {n_id: deque(maxlen=200) for n_id in self.all_possible_neighbors}
        for n_id in self.all_possible_neighbors:
            sub = self.create_subscription(Odometry, f"/usv_{n_id}/odom", lambda msg, nid=n_id: self.neighbor_odom_callback(nid, msg), 10)
            self.neighbor_odom_subscribers.append(sub)

        self.neighbor_decision_subscribers = []
        for n_id in self.all_possible_neighbors:
            sub = self.create_subscription(Decision, f"/usv_{n_id}/topology_decision", self.neighbor_decision_callback, 10)
            self.neighbor_decision_subscribers.append(sub)

        self.neighbor_topology_subscribers = []
        for n_id in self.all_possible_neighbors:
            sub = self.create_subscription(
                TopologicalPathArray,
                f"/usv_{n_id}/topological_path",
                lambda msg, nid=n_id: self.neighbor_topology_callback(nid, msg),
                10
            )
            self.neighbor_topology_subscribers.append(sub)
            
        self.final_goal_pub = self.create_publisher(Point, 'final_goal', 10)
        self.final_goal_timer = self.create_timer(1.0, self.publish_final_goal)
        
        self.pending_mode_timer = self.create_timer(0.5, self.check_pending_mode_switch)
            
        self.get_logger().info("Distributed Smoother Node initialized. Waiting for paths...")

    def _load_debug_env(self):
        debug_url = "http://127.0.0.1:7777/event"
        debug_session = "ros-prm-visualization"
        try:
            with open("/home/lu/paper2/.dbg/ros-prm-visualization.env", "r", encoding="utf-8") as env_file:
                for line in env_file:
                    line = line.strip()
                    if line.startswith("DEBUG_SERVER_URL="):
                        debug_url = line.split("=", 1)[1]
                    elif line.startswith("DEBUG_SESSION_ID="):
                        debug_session = line.split("=", 1)[1]
        except OSError:
            pass
        return debug_url, debug_session

    def _report_debug(self, hypothesis_id, location, msg, data):
        payload = {
            "sessionId": self._debug_session,
            "runId": "pre-fix",
            "hypothesisId": hypothesis_id,
            "location": location,
            "msg": msg,
            "data": data,
        }
        try:
            req = urllib.request.Request(
                self._debug_url,
                data=json.dumps(payload).encode(),
                headers={"Content-Type": "application/json"},
            )
            urllib.request.urlopen(req, timeout=0.2).read()
        except Exception:
            pass

    def update_communication_topology(self):
        return

    def _obs_odom_callback(self, name, msg):
        p = msg.pose.pose.position
        track = self._obstacle_tracks.get(name)
        if track is not None:
            track['pos'] = np.array([p.x, p.y], dtype=float)
            track['stamp'] = time.time()

    def _prune_obstacle_tracks(self):
        now_t = time.time()
        for _, track in list(self._obstacle_tracks.items()):
            if track['pos'] is not None and now_t - track['stamp'] > self._obstacle_timeout:
                track['pos'] = None

    def _get_active_obstacles(self):
        now_t = time.time()
        active = []
        for i, name in enumerate(self.dynamic_obstacle_names):
            track = self._obstacle_tracks.get(name)
            if track is None or track['pos'] is None:
                continue
            if now_t - track['stamp'] > self._obstacle_timeout:
                continue
            radius = float(self.dynamic_obstacle_radii[i]) if i < len(self.dynamic_obstacle_radii) else 3.0
            active.append((track['pos'][0], track['pos'][1], radius, "dynamic", name))
        for idx, (sx, sy, sr) in enumerate(self.static_obstacles):
            active.append((sx, sy, sr, "static", f"static_{idx}"))
        return active

    def _get_non_neighbor_vessels(self):
        if self.my_position is None:
            return []
        vessels = []
        for n_id in self.all_possible_neighbors:
            if n_id in self.active_neighbors:
                continue
            history = self.neighbor_odom_history.get(n_id)
            if not history:
                continue
            pos = history[-1][1]
            dist = float(np.linalg.norm(self.my_position[:2] - pos[:2]))
            if dist > self.usv_perception_radius:
                continue
            vx, vy = 0.0, 0.0
            if len(history) >= 2:
                t0, p0 = history[-2]
                t1, p1 = history[-1]
                dt_v = t1 - t0
                if dt_v > 1e-6:
                    vx = float((p1[0] - p0[0]) / dt_v)
                    vy = float((p1[1] - p0[1]) / dt_v)
            vessels.append((float(pos[0]), float(pos[1]), vx, vy, dist, n_id))
        return vessels

    def non_neighbor_vessel_cost(self, path_msg):
        vessels = self._get_non_neighbor_vessels()
        if not vessels:
            return 0.0

        my_points = self.sample_path_xy(path_msg)
        if len(my_points) < 2:
            return 0.0

        total_cost = 0.0
        my_heading = self.heading_from_points(my_points)
        n_samples = len(my_points)
        predict_horizon = 10.0

        for ox, oy, vvx, vvy, _, _ in vessels:
            other_pos = np.array([ox, oy], dtype=float)
            other_vel = np.array([vvx, vvy], dtype=float)
            other_speed = float(np.linalg.norm(other_vel))

            predicted = np.zeros((n_samples, 2), dtype=float)
            for h in range(n_samples):
                t = (h / max(n_samples - 1, 1)) * predict_horizon
                predicted[h] = other_pos + t * other_vel

            sync_dists = np.linalg.norm(my_points - predicted, axis=1)
            min_sync = float(np.min(sync_dists))
            hard_radius = 2.0 * self.usv_collision_radius + self.vessel_safety_margin
            soft_radius = hard_radius + self.spatiotemporal_soft_margin

            if min_sync < hard_radius:
                total_cost += self.collision_penalty
                continue

            if other_speed < 0.05:
                continue

            other_heading = float(np.arctan2(vvy, vvx))
            heading_delta = abs(self.wrap_angle(my_heading - other_heading))
            rel_vec = other_pos - my_points[0]
            rel_bearing_me = self.wrap_angle(np.arctan2(rel_vec[1], rel_vec[0]) - my_heading)

            if abs(heading_delta - np.pi) < self.head_on_heading_tol and abs(rel_bearing_me) < self.head_on_bearing_tol:
                my_dir = my_points[1] - my_points[0]
                signed_side = my_dir[0] * rel_vec[1] - my_dir[1] * rel_vec[0]
                if signed_side <= 0.0:
                    total_cost += self.usv_non_neighbor_colregs_gain
            elif abs(rel_bearing_me) > 1e-3:
                if rel_bearing_me > 0.0:
                    total_cost += self.usv_non_neighbor_colregs_gain

            total_cost += self.soft_conflict_gain / max(min_sync - hard_radius + 1.0, 1.0) if min_sync < soft_radius else 0.0

        return total_cost

    def _update_exit_state(self):
        clearance = self.min_obstacle_clearance()
        if np.isfinite(clearance) and clearance > self.exit_clearance_margin:
            self.exit_hold_count += 1
        else:
            self.exit_hold_count = 0
        self.has_exited_avoidance_region = self.exit_hold_count >= self.exit_hold_count_required

    def goal_callback(self, msg):
        self.global_target = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.get_logger().info(f"更新全局目标点: {self.global_target}")

    def publish_final_goal(self):
        target_now = self.get_current_target()
        goal_x = target_now[0] + self.offsets[0]
        goal_y = target_now[1] + self.offsets[1]
        goal_z = target_now[2] + self.offsets[2]
        if self.has_exited_avoidance_region and self.my_position is not None:
            goal_x = float(self.my_position[0])
            goal_y = float(self.my_position[1])
            goal_z = float(self.my_position[2])
        msg = Point()
        msg.x = float(goal_x)
        msg.y = float(goal_y)
        msg.z = float(goal_z)
        self.final_goal_pub.publish(msg)

    def mode_callback(self, msg):
        new_mode = msg.data
        previous_requested = self.requested_mode
        self.requested_mode = new_mode
        if new_mode == "FORMATION":
            if self.current_mode != "FORMATION":
                self.get_logger().info(f"感知层下达模式切换指令: {self.current_mode} -> FORMATION")
                self.current_mode = "FORMATION"
                self.reset_game_state()
            return

        if self.current_mode == "AVOIDANCE":
            return

        if self.current_topo_paths:
            self.get_logger().info("感知层请求进入 AVOIDANCE，候选拓扑已就绪，切换到候选选择模式。")
            self.current_mode = "AVOIDANCE"
            self.re_evaluate_game()
        elif previous_requested != "AVOIDANCE":
            self.get_logger().info(
                "感知层请求进入 AVOIDANCE，候选拓扑暂未就绪，等待 PRM 产出路径后自动切换。",
                throttle_duration_sec=2.0
            )
    
    def check_pending_mode_switch(self):
        if self.requested_mode != "AVOIDANCE":
            return
        if self.current_mode == "AVOIDANCE":
            return
        if not self.current_topo_paths:
            return
        self.get_logger().info("pending_mode_timer: 候选拓扑已就绪，执行延迟的 AVOIDANCE 切换。")
        self.current_mode = "AVOIDANCE"
        self.re_evaluate_game()
    
    def get_current_target(self):
        """Get the current global target."""
        return np.array(self.global_target, dtype=float)
    
    def my_odom_callback(self, msg):
        p = msg.pose.pose.position
        self.my_position = np.array([p.x, p.y, p.z], dtype=float)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.my_heading = float(np.arctan2(siny_cosp, cosy_cosp))
        self._update_exit_state()
    
    def neighbor_odom_callback(self, neighbor_id, msg):
        p = msg.pose.pose.position
        t = self.get_clock().now().nanoseconds * 1e-9
        self.neighbor_odom_history[neighbor_id].append((t, np.array([p.x, p.y, p.z], dtype=float)))
    
    def get_neighbor_position(self, neighbor_id, delay=None):
        history = self.neighbor_odom_history.get(neighbor_id, None)
        if not history:
            return None
        
        target_delay = delay if delay is not None else self.comm_delay_sec
        
        if target_delay <= 1e-6:
            return history[-1][1]
            
        now_t = self.get_clock().now().nanoseconds * 1e-9
        target_t = now_t - target_delay
        
        for t, p in reversed(history):
            if t <= target_t:
                return p
        return history[0][1]
    
    def min_obstacle_clearance(self):
        if self.my_position is None:
            return float('inf')
        min_clearance = float('inf')
        for ox, oy, r, kind, _ in self._get_active_obstacles():
            center = np.array([ox, oy], dtype=float)
            radius = r + self.usv_collision_radius + (self.static_obstacle_safety_margin if kind == "static" else self.vessel_safety_margin)
            dist = float(np.linalg.norm(self.my_position[:2] - center))
            min_clearance = min(min_clearance, dist - radius)
        return min_clearance

    def obstacle_relevance(self, obstacle, position=None, heading=None, path_msg=None):
        if position is None:
            position = self.my_position
        if position is None:
            return None

        pos = np.array(position[:2], dtype=float)
        center = np.array([obstacle[0], obstacle[1]], dtype=float)
        kind = obstacle[3]
        name = obstacle[4]
        effective_r = obstacle[2] + self.usv_collision_radius + (self.static_obstacle_safety_margin if kind == "static" else self.vessel_safety_margin)
        rel = center - pos
        dist = float(np.linalg.norm(rel))
        clearance = dist - effective_r

        if path_msg is not None and self.is_valid_path(path_msg):
            points = self.sample_path_xy(path_msg)
            if len(points) >= 2:
                heading = self.heading_from_points(points)
                path_clearance = float(np.min(np.linalg.norm(points - center, axis=1) - effective_r))
            else:
                path_clearance = clearance
        else:
            path_clearance = clearance

        if heading is None:
            heading = self.my_heading
        if heading is None:
            bearing = 0.0
            forward = True
        else:
            bearing = float(abs(self.relative_bearing(heading, pos, center)))
            forward = bearing <= np.deg2rad(95.0)

        relevant = forward and (clearance < self.local_avoid_trigger_margin or path_clearance < self.local_avoid_trigger_margin)
        return {
            'name': name,
            'kind': kind,
            'center_x': float(center[0]),
            'center_y': float(center[1]),
            'raw_radius': float(obstacle[2]),
            'effective_radius': float(effective_r),
            'clearance': clearance,
            'path_clearance': path_clearance,
            'bearing': bearing,
            'forward': forward,
            'relevant': relevant,
        }

    def current_relevant_obstacle(self, position=None, heading=None, path_msg=None):
        best = None
        for obstacle in self._get_active_obstacles():
            info = self.obstacle_relevance(obstacle, position=position, heading=heading, path_msg=path_msg)
            if info is None or not info['relevant']:
                continue
            if best is None or info['path_clearance'] < best['path_clearance']:
                best = info
        return best

    def obstacle_forward_disc_membership(self, obstacle_info, position, heading=None):
        if position is None:
            return False
        if heading is None:
            heading = self.my_heading
        if heading is None:
            return False
        pos = np.array(position[:2], dtype=float)
        heading_vec = np.array([np.cos(heading), np.sin(heading)], dtype=float)
        lateral_vec = np.array([-heading_vec[1], heading_vec[0]], dtype=float)
        center = np.array([obstacle_info['center_x'], obstacle_info['center_y']], dtype=float)
        rel = center - pos
        z = float(np.dot(rel, heading_vec))
        lateral = abs(float(np.dot(rel, lateral_vec)))
        disc_radius = obstacle_info['raw_radius'] + self.usv_collision_radius + self.vessel_safety_margin
        z_start = -disc_radius
        z_end = obstacle_info['clearance'] + disc_radius + self.shared_obstacle_z_margin
        return lateral <= disc_radius and z_start <= z <= z_end

    def neighbor_shares_relevant_obstacle(self, neighbor_id):
        my_obstacle = self.current_relevant_obstacle()
        if my_obstacle is None or self.my_position is None:
            return False
        n_pos = self.get_neighbor_position(neighbor_id, delay=0.0)
        if n_pos is None:
            return False
        neighbor_obstacle = self.current_relevant_obstacle(position=n_pos, heading=self.my_heading)
        if neighbor_obstacle is None or neighbor_obstacle['name'] != my_obstacle['name']:
            return False
        if not self.obstacle_forward_disc_membership(my_obstacle, self.my_position, self.my_heading):
            return False
        if not self.obstacle_forward_disc_membership(my_obstacle, n_pos, self.my_heading):
            return False

        heading_vec = np.array([np.cos(self.my_heading), np.sin(self.my_heading)], dtype=float)
        neighbor_z = float(np.dot(n_pos[:2] - self.my_position[:2], heading_vec))
        if neighbor_z > self.shared_obstacle_z_margin:
            return False
        return abs(neighbor_obstacle['path_clearance'] - my_obstacle['path_clearance']) <= self.shared_obstacle_distance_margin

    def emergency_lock_needed(self):
        if self.min_obstacle_clearance() < self.emergency_lock_margin:
            return True
        if not self.current_topo_paths:
            return False
        safe_options = 0
        for topo_path in self.current_topo_paths:
            if not self.is_valid_path(topo_path.path):
                continue
            if self.path_min_clearance(topo_path.path) > self.obstacle_critical_clearance:
                safe_options += 1
        return safe_options <= 1 and self.min_obstacle_clearance() < self.obstacle_priority_distance

    def cancel_lock_timer(self):
        if self.lock_timer:
            self.lock_timer.cancel()
            self.lock_timer = None

    def reset_game_state(self):
        self.game_round = 0
        self.game_locked = False
        self.my_last_choice = None
        self.my_last_signature = None
        self.choice_stability_count = 0
        self.last_candidate_change_time = -1.0
        self.last_discussion_neighbors = set()
        self.cancel_lock_timer()

    def lock_current_choice(self, reason):
        if self.my_last_choice is None or self.my_last_choice < 0 or not self.current_topo_paths:
            return

        self.game_locked = True
        self.cancel_lock_timer()
        self.avoidance_lock_until = self.get_clock().now().nanoseconds * 1e-9 + self.avoidance_decision_hold_sec
        self.get_logger().info(
            f"USV {self.my_id} 锁定路径 {self.my_last_choice}: {reason}"
        )
        self.publish_current_decision(self.my_last_choice)
        self.publish_path(self.current_topo_paths[self.my_last_choice].path)

    def publish_current_decision(self, choice_idx):
        msg = Decision()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.agent_id = self.my_id
        
        if choice_idx != -1 and self.current_topo_paths:
            msg.h_signature = list(self.current_topo_paths[choice_idx].h_signature)
            msg.planned_path = self.current_topo_paths[choice_idx].path
            msg.base_cost = self.nav_costs[choice_idx]
            msg.is_locked = self.game_locked
            self.my_last_signature = self.topology_key_from_signature(msg.h_signature)
        else:
            msg.h_signature = []
            msg.planned_path = Path()
            msg.base_cost = 999999.0
            msg.is_locked = False
            
        self.decision_publisher_.publish(msg)

    def is_valid_path(self, path_msg):
        return path_msg is not None and len(path_msg.poses) >= 2

    def sample_path_xy(self, path_msg):
        return compute_sample_path_xy(path_msg, self.path_sample_count)

    def topology_key_from_signature(self, h_signature):
        return tuple(round(float(v), 6) for v in h_signature)

    def path_index_from_signature(self, topo_paths, h_signature):
        if topo_paths is None:
            return None
        target_key = self.topology_key_from_signature(h_signature)
        for idx, topo_path in enumerate(topo_paths):
            if self.topology_key_from_signature(topo_path.h_signature) == target_key:
                return idx
        return None

    def current_signature_list(self, topo_paths):
        if topo_paths is None:
            return []
        return [self.topology_key_from_signature(topo_path.h_signature) for topo_path in topo_paths]

    def heading_from_points(self, points):
        if len(points) < 2:
            return 0.0
        direction = points[min(3, len(points) - 1)] - points[0]
        if np.linalg.norm(direction) < 1e-6:
            direction = points[-1] - points[0]
        return float(np.arctan2(direction[1], direction[0]))

    def wrap_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def relative_bearing(self, heading, source, target):
        vec = target - source
        return self.wrap_angle(np.arctan2(vec[1], vec[0]) - heading)

    def encounter_type(self, my_points, other_points):
        my_heading = self.heading_from_points(my_points)
        other_heading = self.heading_from_points(other_points)
        heading_delta = abs(self.wrap_angle(my_heading - other_heading))

        rel_bearing_me = self.relative_bearing(my_heading, my_points[0], other_points[0])
        rel_bearing_other = self.relative_bearing(other_heading, other_points[0], my_points[0])

        if abs(heading_delta - np.pi) < self.head_on_heading_tol and abs(rel_bearing_me) < self.head_on_bearing_tol:
            return "head_on", rel_bearing_me, rel_bearing_other
        if heading_delta < self.overtaking_heading_tol and abs(rel_bearing_me) > self.overtaking_bearing:
            return "overtaking", rel_bearing_me, rel_bearing_other
        return "crossing", rel_bearing_me, rel_bearing_other

    def path_length(self, path_msg):
        return compute_path_length(path_msg)

    def path_min_clearance(self, path_msg):
        active_obs = self._get_active_obstacles()
        if not active_obs:
            return float('inf')
        points = self.sample_path_xy(path_msg)
        if len(points) == 0:
            return float('inf')
        best_clearance = float('inf')
        for ox, oy, r, kind, _ in active_obs:
            center = np.array([ox, oy], dtype=float)
            radius = r + self.usv_collision_radius + (self.static_obstacle_safety_margin if kind == "static" else self.vessel_safety_margin)
            clearance = float(np.min(np.linalg.norm(points - center, axis=1) - radius))
            best_clearance = min(best_clearance, clearance)
        return best_clearance

    def path_obstacle_cost(self, path_msg):
        active_obs = self._get_active_obstacles()
        if not active_obs:
            return 0.0

        points = self.sample_path_xy(path_msg)
        if len(points) == 0:
            return 0.0

        relevant_obs = [obs for obs in active_obs if self.obstacle_relevance(obs, path_msg=path_msg) is not None and self.obstacle_relevance(obs, path_msg=path_msg)['relevant']]

        best_clearance = float('inf')
        best_relevant_clearance = float('inf')
        for ox, oy, r, kind, obs_name in active_obs:
            center = np.array([ox, oy], dtype=float)
            radius = r + self.usv_collision_radius + (self.static_obstacle_safety_margin if kind == "static" else self.vessel_safety_margin)
            distances = np.linalg.norm(points - center, axis=1) - radius
            clearance = float(np.min(distances))
            best_clearance = min(best_clearance, clearance)
            if any(obs[4] == obs_name for obs in relevant_obs):
                best_relevant_clearance = min(best_relevant_clearance, clearance)

        if best_clearance < 0.0:
            return self.collision_penalty
        if best_clearance < self.obstacle_critical_clearance:
            critical_gap = max(best_clearance + 1.0, 0.25)
            return self.collision_penalty * 0.45 + self.collision_penalty * (self.obstacle_critical_clearance - best_clearance) / self.obstacle_critical_clearance + self.soft_conflict_gain * 20.0 / critical_gap
        if best_clearance < self.obstacle_priority_distance:
            normalized = (self.obstacle_priority_distance - best_clearance) / max(self.obstacle_priority_distance, 1.0)
            priority_cost = self.collision_penalty * 0.08 * normalized * normalized + self.soft_conflict_gain * 8.0 / max(best_clearance, 1.0)
            if best_relevant_clearance < float('inf'):
                priority_cost += self.collision_penalty * 0.04 * normalized
            return priority_cost
        if best_relevant_clearance < float('inf'):
            return self.soft_conflict_gain * 3.0 / max(best_relevant_clearance, 1.0)
        return self.soft_conflict_gain * 0.15 / max(best_clearance, 1.0)

    def publish_path(self, path_msg):
        output = Path()
        output.header.frame_id = "odom"
        output.header.stamp = self.get_clock().now().to_msg()
        max_points = max(self.formation_path_points, self.path_sample_count)
        poses = list(path_msg.poses)
        if len(poses) >= 2 and len(poses) < max_points:
            dense_poses = []
            for idx in range(len(poses) - 1):
                p0 = poses[idx].pose.position
                p1 = poses[idx + 1].pose.position
                dense_poses.append(poses[idx])
                steps = max(1, int(np.ceil(max_points / max(len(poses) - 1, 1))))
                for step in range(1, steps):
                    t = step / steps
                    pose = PoseStamped()
                    pose.header = output.header
                    pose.pose.position.x = float(p0.x + t * (p1.x - p0.x))
                    pose.pose.position.y = float(p0.y + t * (p1.y - p0.y))
                    pose.pose.position.z = 0.0
                    dense_poses.append(pose)
            dense_poses.append(poses[-1])
            poses = dense_poses[:max_points]
        else:
            poses = poses[:max_points]
        for pose_in in poses:
            pose = PoseStamped()
            pose.header = output.header
            pose.pose = pose_in.pose
            pose.pose.position.z = 0.0
            output.poses.append(pose)
        self.smooth_publisher.publish(output)

    def publish_fallback_straight_path(self):
        my_pos = self.my_position
        if my_pos is None:
            return
        target_now = self.get_current_target()
        goal = np.array([
            target_now[0] + self.offsets[0],
            target_now[1] + self.offsets[1],
            target_now[2] + self.offsets[2],
        ], dtype=float)
        if np.linalg.norm(my_pos - goal) < self.stop_radius:
            return
        smooth_path_msg = Path()
        smooth_path_msg.header.frame_id = "odom"
        smooth_path_msg.header.stamp = self.get_clock().now().to_msg()
        N = max(2, self.formation_path_points)
        for i in range(N):
            t = i / (N - 1)
            pose = PoseStamped()
            pose.header = smooth_path_msg.header
            pose.pose.position.x = my_pos[0] + t * (goal[0] - my_pos[0])
            pose.pose.position.y = my_pos[1] + t * (goal[1] - my_pos[1])
            pose.pose.position.z = 0.0
            smooth_path_msg.poses.append(pose)
        self.smooth_publisher.publish(smooth_path_msg)
        self.get_logger().info(
            f"USV {self.my_id} PRM 空路径降级: 生成直线轨迹 "
            f"({my_pos[0]:.1f},{my_pos[1]:.1f}) -> ({goal[0]:.1f},{goal[1]:.1f})"
        )

    def timed_path_points(self, path_msg):
        points = self.sample_path_xy(path_msg)
        if len(points) == 0:
            return points, np.zeros(0, dtype=float)
        if len(points) == 1:
            return points, np.zeros(1, dtype=float)
        seg_lengths = np.linalg.norm(points[1:] - points[:-1], axis=1)
        cum_lengths = np.concatenate(([0.0], np.cumsum(seg_lengths)))
        total = max(float(cum_lengths[-1]), 1e-6)
        times = cum_lengths / total * self.spatiotemporal_horizon_sec
        return points, times

    def resample_by_time(self, points, times, query_times):
        if len(points) == 0:
            return np.zeros((0, 2), dtype=float)
        if len(points) == 1:
            return np.repeat(points, len(query_times), axis=0)
        xs = np.interp(query_times, times, points[:, 0])
        ys = np.interp(query_times, times, points[:, 1])
        return np.column_stack((xs, ys))

    def pairwise_closing_metrics(self, my_sync, other_sync, query_times):
        rel = other_sync - my_sync
        distances = np.linalg.norm(rel, axis=1)
        closest_idx = int(np.argmin(distances))
        if len(query_times) < 2:
            return closest_idx, float(distances[closest_idx]), 0.0, float(query_times[closest_idx])
        dt = max(float(query_times[1] - query_times[0]), 1e-6)
        my_vel = np.gradient(my_sync, dt, axis=0)
        other_vel = np.gradient(other_sync, dt, axis=0)
        rel_vel = other_vel - my_vel
        rel_at_closest = rel[closest_idx]
        dist_at_closest = max(float(distances[closest_idx]), 1e-6)
        closing_speed = -float(np.dot(rel_at_closest, rel_vel[closest_idx])) / dist_at_closest
        return closest_idx, dist_at_closest, closing_speed, float(query_times[closest_idx])

    def pairwise_collision_cost(self, my_path, other_path):
        my_points, my_times = self.timed_path_points(my_path)
        other_points, other_times = self.timed_path_points(other_path)
        if len(my_points) == 0 or len(other_points) == 0:
            return 0.0

        query_times = np.linspace(0.0, self.spatiotemporal_horizon_sec, self.path_sample_count)
        my_sync = self.resample_by_time(my_points, my_times, query_times)
        other_sync = self.resample_by_time(other_points, other_times, query_times)
        closest_idx, min_sync_dist, closing_speed, time_at_closest = self.pairwise_closing_metrics(my_sync, other_sync, query_times)
        hard_radius = 2.0 * self.usv_collision_radius + self.vessel_safety_margin
        soft_radius = hard_radius + self.spatiotemporal_soft_margin
        is_closing = closing_speed > self.relative_closing_speed_threshold
        immediate_overlap = min_sync_dist < hard_radius and time_at_closest <= self.min_pairwise_ttc_sec

        if min_sync_dist < hard_radius:
            if immediate_overlap:
                return self.collision_penalty
            if is_closing:
                return self.collision_penalty * 0.45
            return self.soft_conflict_gain * 2.0 / max(min_sync_dist, 1.0)
        if min_sync_dist < soft_radius and is_closing:
            normalized = (soft_radius - min_sync_dist) / max(soft_radius - hard_radius, 1.0)
            return self.collision_penalty * 0.025 * normalized * normalized + self.soft_conflict_gain * 3.0 / max(min_sync_dist - hard_radius + 0.5, 0.5)
        return 0.0

    def pairwise_colregs_cost(self, my_path, other_path):
        my_points = self.sample_path_xy(my_path)
        other_points = self.sample_path_xy(other_path)
        if len(my_points) < 2 or len(other_points) < 2:
            return 0.0

        encounter, rel_bearing_me, _ = self.encounter_type(my_points, other_points)
        sync_distances = np.linalg.norm(my_points - other_points, axis=1)
        closest_idx = int(np.argmin(sync_distances))

        my_idx = min(max(closest_idx, 1), len(my_points) - 1)
        other_idx = min(max(closest_idx, 1), len(other_points) - 1)

        my_dir = my_points[my_idx] - my_points[my_idx - 1]
        rel_vec = other_points[closest_idx] - my_points[closest_idx]
        signed_side = my_dir[0] * rel_vec[1] - my_dir[1] * rel_vec[0]

        if encounter == "head_on":
            return 0.0 if signed_side > 0.0 else self.colregs_penalty

        if encounter == "crossing":
            if rel_bearing_me < 0.0:
                return 0.0
            my_progress = my_idx / max(len(my_points) - 1, 1)
            other_progress = other_idx / max(len(other_points) - 1, 1)
            return 0.0 if my_progress > other_progress + self.crossing_progress_margin else self.colregs_penalty

        hard_radius = 2.0 * self.usv_collision_radius + self.vessel_safety_margin
        clearance = float(np.min(sync_distances)) - hard_radius
        return max(0.0, self.colregs_penalty * 0.5 * (self.spatiotemporal_soft_margin - clearance))

    def joint_cost_for_pair(self, my_idx, other_idx, neighbor_id, neighbor_paths, neighbor_nav_costs):
        my_topo = self.current_topo_paths[my_idx]
        other_topo = neighbor_paths[other_idx]

        my_cost = self.nav_costs[my_idx] + self.path_obstacle_cost(my_topo.path)
        other_cost = neighbor_nav_costs[other_idx]

        if self.my_last_choice is not None and my_idx != self.my_last_choice:
            my_cost += self.consistency_penalty + self.path_switch_penalty

        neighbor_last_idx = None
        neighbor_decision = self.neighbor_decisions.get(neighbor_id, {})
        if neighbor_decision.get('h_sig', []):
            neighbor_last_idx = self.path_index_from_signature(neighbor_paths, neighbor_decision['h_sig'])
        if neighbor_last_idx is not None and other_idx != neighbor_last_idx:
            other_cost += self.consistency_penalty

        collision_cost = self.pairwise_collision_cost(my_topo.path, other_topo.path)
        my_cost += collision_cost + self.pairwise_colregs_cost(my_topo.path, other_topo.path)
        other_cost += collision_cost + self.pairwise_colregs_cost(other_topo.path, my_topo.path)

        return my_cost, other_cost

    def predict_neighbor_reference_path(self, neighbor_id):
        neighbor_info = self.neighbor_topo_sets.get(neighbor_id, None)
        neighbor_decision = self.neighbor_decisions.get(neighbor_id, {})

        neighbor_has_single_option = False
        if neighbor_info and neighbor_info.get('paths'):
            neighbor_has_single_option = len(neighbor_info['paths']) == 1

        decision_path = neighbor_decision.get('path', None)
        if decision_path is not None and getattr(decision_path, 'poses', None):
            return decision_path, neighbor_has_single_option

        if not neighbor_info or not neighbor_info['paths']:
            return None, False

        if neighbor_has_single_option:
            return neighbor_info['paths'][0].path, True

        neighbor_nav_costs = neighbor_info.get('nav_costs', [])
        best_idx = int(np.argmin(neighbor_nav_costs)) if neighbor_nav_costs else 0
        return neighbor_info['paths'][best_idx].path, False

    def should_yield_to_single_route(self, my_path, neighbor_path):
        my_points = self.sample_path_xy(my_path)
        other_points = self.sample_path_xy(neighbor_path)
        if len(my_points) < 2 or len(other_points) < 2:
            return False

        min_sync_dist = float(np.min(np.linalg.norm(my_points - other_points, axis=1)))
        hard_radius = 2.0 * self.usv_collision_radius + self.vessel_safety_margin
        return min_sync_dist < hard_radius + self.spatiotemporal_soft_margin

    def solve_two_agent_equilibrium(self, neighbor_id):
        if not self.neighbor_within_discussion_distance(neighbor_id):
            return None
        neighbor_info = self.neighbor_topo_sets.get(neighbor_id, None)
        if not neighbor_info or not neighbor_info['paths'] or not self.current_topo_paths:
            return None

        neighbor_paths = neighbor_info['paths']
        neighbor_nav_costs = neighbor_info['nav_costs']
        if not neighbor_nav_costs:
            return None

        neighbor_decision = self.neighbor_decisions.get(neighbor_id, {})
        neighbor_locked = bool(neighbor_decision.get('is_locked', False))
        neighbor_locked_idx = None
        if neighbor_decision.get('h_sig', []):
            neighbor_locked_idx = self.path_index_from_signature(neighbor_paths, neighbor_decision['h_sig'])

        candidate_pairs = []
        for my_idx in range(len(self.current_topo_paths)):
            if neighbor_locked and neighbor_locked_idx is not None:
                other_indices = [neighbor_locked_idx]
            else:
                other_indices = range(len(neighbor_paths))

            for other_idx in other_indices:
                my_cost, other_cost = self.joint_cost_for_pair(
                    my_idx, other_idx, neighbor_id, neighbor_paths, neighbor_nav_costs
                )
                candidate_pairs.append((my_idx, other_idx, my_cost, other_cost))

        if not candidate_pairs:
            return None

        equilibria = []
        if not neighbor_locked or neighbor_locked_idx is None:
            for my_idx, other_idx, my_cost, other_cost in candidate_pairs:
                my_best_response = True
                for alt_my in range(len(self.current_topo_paths)):
                    alt_cost, _ = self.joint_cost_for_pair(
                        alt_my, other_idx, neighbor_id, neighbor_paths, neighbor_nav_costs
                    )
                    if alt_cost + 1e-6 < my_cost:
                        my_best_response = False
                        break

                if not my_best_response:
                    continue

                other_best_response = True
                for alt_other in range(len(neighbor_paths)):
                    _, alt_other_cost = self.joint_cost_for_pair(
                        my_idx, alt_other, neighbor_id, neighbor_paths, neighbor_nav_costs
                    )
                    if alt_other_cost + 1e-6 < other_cost:
                        other_best_response = False
                        break

                if other_best_response:
                    social_cost = my_cost + other_cost
                    ordered_pair = (my_idx, other_idx) if self.my_id < neighbor_id else (other_idx, my_idx)
                    equilibria.append((social_cost, ordered_pair, my_idx, other_idx, my_cost, other_cost))

        if equilibria:
            equilibria.sort(key=lambda item: (item[0], item[1]))
            _, _, best_my_idx, best_other_idx, best_my_cost, best_other_cost = equilibria[0]
            return best_my_idx, best_my_cost, best_other_idx, best_other_cost, True

        candidate_pairs.sort(key=lambda item: (item[2] + item[3], (item[0], item[1]) if self.my_id < neighbor_id else (item[1], item[0])))
        best_my_idx, best_other_idx, best_my_cost, best_other_cost = candidate_pairs[0]
        return best_my_idx, best_my_cost, best_other_idx, best_other_cost, False

    def neighbor_within_discussion_distance(self, neighbor_id):
        n_pos = self.get_neighbor_position(neighbor_id, delay=0.0)
        if n_pos is None or self.my_position is None:
            return False
        dist = float(np.linalg.norm(self.my_position[:2] - n_pos[:2]))
        return dist <= self.neighbor_discussion_distance and self.neighbor_has_path_conflict(neighbor_id)

    def neighbor_has_path_conflict(self, neighbor_id):
        if not self.current_topo_paths:
            return False
        neighbor_info = self.neighbor_topo_sets.get(neighbor_id, None)
        if not neighbor_info or not neighbor_info.get('paths'):
            return True

        neighbor_paths = neighbor_info['paths']
        hard_radius = 2.0 * self.usv_collision_radius + self.vessel_safety_margin
        conflict_margin = hard_radius + self.spatiotemporal_soft_margin

        for my_topo in self.current_topo_paths:
            if not self.is_valid_path(my_topo.path):
                continue
            my_points, my_times = self.timed_path_points(my_topo.path)
            if len(my_points) == 0:
                continue

            for nb_topo in neighbor_paths:
                if not self.is_valid_path(nb_topo.path):
                    continue
                nb_points, nb_times = self.timed_path_points(nb_topo.path)
                if len(nb_points) == 0:
                    continue

                query_times = np.linspace(0.0, min(self.spatiotemporal_horizon_sec, self.spatiotemporal_horizon_sec), self.path_sample_count)
                my_sync = self.resample_by_time(my_points, my_times, query_times)
                nb_sync = self.resample_by_time(nb_points, nb_times, query_times)
                min_dist = float(np.min(np.linalg.norm(my_sync - nb_sync, axis=1)))

                if min_dist < conflict_margin:
                    return True

        return False

    def compute_best_choice(self):
        best_cost = float('inf')
        best_choice = -1
        self.last_discussion_neighbors = set()

        for k, topo_path in enumerate(self.current_topo_paths):
            if not self.is_valid_path(topo_path.path):
                continue
            total_cost = self.nav_costs[k] + self.path_obstacle_cost(topo_path.path)

            if self.my_last_choice is not None and k != self.my_last_choice:
                total_cost += self.consistency_penalty + self.path_switch_penalty

            for neighbor_id in self.active_neighbors:
                if not self.neighbor_within_discussion_distance(neighbor_id):
                    continue
                self.last_discussion_neighbors.add(neighbor_id)
                neighbor_path, neighbor_has_single_option = self.predict_neighbor_reference_path(neighbor_id)
                if neighbor_path is None or not neighbor_path.poses:
                    continue

                total_cost += self.pairwise_collision_cost(topo_path.path, neighbor_path)
                total_cost += self.pairwise_colregs_cost(topo_path.path, neighbor_path)
                if neighbor_has_single_option and len(self.current_topo_paths) > 1:
                    if self.should_yield_to_single_route(topo_path.path, neighbor_path):
                        total_cost += self.single_path_yield_penalty

            total_cost += self.non_neighbor_vessel_cost(topo_path.path)

            if total_cost < best_cost:
                best_cost = total_cost
                best_choice = k

        return best_choice, best_cost


    def emergency_lock_choice(self):
        if not self.current_topo_paths:
            return
        best_choice, _ = self.compute_best_choice()
        if best_choice < 0:
            return
        self.my_last_choice = best_choice
        self.lock_current_choice(
            f"进入紧急锁路区 (clearance={self.min_obstacle_clearance():.2f}m)"
        )

    def neighbor_decision_callback(self, msg):
        # 记录邻居的决策和他们算出来的绝对轨迹
        self.neighbor_decisions[msg.agent_id] = {
            'h_sig': msg.h_signature,
            'path': msg.planned_path,
            'base_cost': getattr(msg, 'base_cost', 999999.0),
            'is_locked': getattr(msg, 'is_locked', False)
        }
        if not self.current_topo_paths:
            self.get_logger().info(f"收到邻居 USV {msg.agent_id} 的决策更新，但本艇候选拓扑尚未就绪，先缓存。")
            return
        if self.current_mode != "AVOIDANCE":
            return
        self.get_logger().info(f"收到邻居 USV {msg.agent_id} 的决策更新 (locked={getattr(msg, 'is_locked', False)})! 触发博弈重评估...")
        # 只要有邻居更新了决策，我们就利用缓存的轨迹重新评估最佳响应！
        self.re_evaluate_game()

    def neighbor_topology_callback(self, neighbor_id, msg):
        raw_paths = list(msg.topological_paths)
        new_paths = [p for p in raw_paths if self.is_valid_path(p.path)]
        dropped = len(raw_paths) - len(new_paths)
        if not new_paths and raw_paths:
            self.get_logger().warn(
                f"收到邻居 USV {neighbor_id} 的候选拓扑集全部无效(poses<2)，保留上一帧候选集不更新。"
            )
            return
        if dropped > 0:
            self.get_logger().warn(
                f"收到邻居 USV {neighbor_id} 的候选拓扑集包含无效路径(poses<2)，已丢弃 {dropped}/{len(raw_paths)} 条。"
            )
        new_signatures = self.current_signature_list(new_paths)
        old_signatures = self.current_signature_list(self.neighbor_topo_sets.get(neighbor_id, {}).get('paths', []))
        self.neighbor_topo_sets[neighbor_id] = {
            'paths': new_paths,
            'nav_costs': [self.path_length(topo_path.path) for topo_path in new_paths],
        }

        if new_signatures != old_signatures:
            if not self.current_topo_paths:
                self.get_logger().info(f"收到邻居 USV {neighbor_id} 的候选拓扑集，但本艇候选拓扑尚未就绪，先缓存。")
                return
            if self.current_mode != "AVOIDANCE":
                return
            self.get_logger().info(
                f"收到邻居 USV {neighbor_id} 的候选拓扑集: count={len(new_paths)}，触发非合作重评估。"
            )
            self.re_evaluate_game()

    def path_callback(self,msg):
        # #region debug-point D:smoother-paths
        self._report_debug(
            "D",
            "smoother_node.py:path_callback",
            f"[DEBUG] smoother received topological paths for {self.agent_name}",
            {"raw_count": len(msg.topological_paths), "mode": self.current_mode, "requested_mode": self.requested_mode},
        )
        # #endregion
        old_signatures = self.current_signature_list(self.current_topo_paths)
        raw_paths = list(msg.topological_paths)
        filtered_paths = [p for p in raw_paths if self.is_valid_path(p.path)]
        dropped = len(raw_paths) - len(filtered_paths)
        if not filtered_paths and raw_paths:
            self.get_logger().warn(
                f"USV {self.my_id} 收到的 PRM 候选集全部无效(poses<2)，保留上一帧候选集继续执行。"
            )
            if self.current_topo_paths and self.my_last_choice is not None:
                idx = int(self.my_last_choice)
                if 0 <= idx < len(self.current_topo_paths):
                    self.publish_path(self.current_topo_paths[idx].path)
                    return
            self.publish_fallback_straight_path()
            return
        if not filtered_paths:
            self.current_topo_paths = []
            self.nav_costs = []
            if self.requested_mode == "AVOIDANCE":
                self.get_logger().info("感知层请求避障，但本艇尚未收到 V-PRM 候选，继续按直线编队前进。")
            self.publish_fallback_straight_path()
            return

        self.get_logger().info(f"Received {self.my_id} topological paths. Evaluating base lengths (Fast Mode)...")
        self.current_topo_paths = filtered_paths
        if dropped > 0:
            self.get_logger().warn(
                f"USV {self.my_id} 收到的 PRM 候选集中存在无效路径(poses<2)，已丢弃 {dropped}/{len(raw_paths)} 条。"
            )
        new_signatures = self.current_signature_list(self.current_topo_paths)
        self.nav_costs = [self.path_length(topo_path.path) for topo_path in self.current_topo_paths]

        if self.my_last_signature is not None:
            remapped_idx = self.path_index_from_signature(self.current_topo_paths, self.my_last_signature)
            self.my_last_choice = remapped_idx

        if new_signatures != old_signatures:
            self.game_round = 0
            self.game_locked = False
            self.choice_stability_count = 0
            self.last_candidate_change_time = self.get_clock().now().nanoseconds * 1e-9
            self.cancel_lock_timer()
            self.get_logger().info(
                f"USV {self.my_id} 的候选拓扑集合发生变化: old={len(old_signatures)}, new={len(new_signatures)}"
            )

        if self.requested_mode == "AVOIDANCE" and self.current_mode != "AVOIDANCE":
            self.get_logger().info("本艇 V-PRM 候选已到位，AVOIDANCE 请求正式生效，开始选路。")
            self.current_mode = "AVOIDANCE"
            self.re_evaluate_game()
            return

        if self.current_mode == "FORMATION" and self.current_relevant_obstacle() is not None:
            relevant_obstacle = self.current_relevant_obstacle()
            self.get_logger().info(
                f"相关障碍进入局部避障触发区，切换到 AVOIDANCE (obstacle={relevant_obstacle['name']}, clearance={relevant_obstacle['clearance']:.2f}m)。"
            )
            self.current_mode = "AVOIDANCE"
            self.requested_mode = "AVOIDANCE"

        if self.current_mode == "FORMATION":
            self.get_logger().info(f"USV {self.my_id} 当前模式为 FORMATION (无障碍)，启动优化退化，直接生成编队直线轨迹。")
            start_pose = self.current_topo_paths[0].path.poses[0] if self.current_topo_paths[0].path.poses else None
            if self.my_position is not None:
                my_pos = self.my_position
            elif start_pose is not None:
                my_pos = np.array([
                    start_pose.pose.position.x,
                    start_pose.pose.position.y,
                    start_pose.pose.position.z,
                ], dtype=float)
            else:
                return

            target_now = self.get_current_target()
            nominal_goal = np.array([
                target_now[0] + self.offsets[0],
                target_now[1] + self.offsets[1],
                target_now[2] + self.offsets[2],
            ], dtype=float)

            if np.linalg.norm(my_pos - nominal_goal) < self.stop_radius:
                self.get_logger().info(f"USV {self.my_id} 已到达目标区域，启动停车机制！")
                smooth_path_msg = Path()
                smooth_path_msg.header.frame_id = "odom"
                smooth_path_msg.header.stamp = self.get_clock().now().to_msg()
                pose = PoseStamped()
                pose.header = smooth_path_msg.header
                pose.pose.position.x = my_pos[0]
                pose.pose.position.y = my_pos[1]
                pose.pose.position.z = my_pos[2]
                smooth_path_msg.poses.append(pose)
                self.smooth_publisher.publish(smooth_path_msg)
                return

            corr = np.zeros(3, dtype=float)
            cnt = 0
            for n_id in self.active_neighbors:
                n_pos = self.get_neighbor_position(n_id)
                if n_pos is None:
                    continue
                desired_from_n = n_pos + (self.all_offsets[self.my_id] - self.all_offsets[n_id])
                corr += (desired_from_n - my_pos)
                cnt += 1
            if cnt > 0:
                corr = (self.k_form / cnt) * corr
                corr = np.clip(corr, -2.0, 2.0)
            corrected_goal = nominal_goal + corr

            N = max(2, self.formation_path_points)
            smooth_path_msg = Path()
            smooth_path_msg.header.frame_id = "odom"
            smooth_path_msg.header.stamp = self.get_clock().now().to_msg()
            for i in range(N):
                t = i / (N - 1) if N > 1 else 1.0
                pose = PoseStamped()
                pose.header.stamp = smooth_path_msg.header.stamp
                pose.pose.position.x = my_pos[0] + t * (corrected_goal[0] - my_pos[0])
                pose.pose.position.y = my_pos[1] + t * (corrected_goal[1] - my_pos[1])
                pose.pose.position.z = 0.0
                smooth_path_msg.poses.append(pose)
            self.smooth_publisher.publish(smooth_path_msg)
            return

        if self.current_mode == "AVOIDANCE" and len(self.current_topo_paths) == 1:
            self.my_last_choice = 0
            self.choice_stability_count = self.route_lock_repeat_count
            self.publish_current_decision(0)
            self.publish_path(self.current_topo_paths[0].path)
            return
        
        if self.current_mode == "AVOIDANCE" and self.emergency_lock_needed():
            self.emergency_lock_choice()
            return

        if self.current_mode == "AVOIDANCE" and self.current_relevant_obstacle() is None and not self.last_discussion_neighbors:
            if self.my_last_choice is not None:
                idx = int(self.my_last_choice)
                if 0 <= idx < len(self.current_topo_paths):
                    self.publish_path(self.current_topo_paths[idx].path)
                    return

        # 立刻开始博弈决策
        self.re_evaluate_game()
            
    def re_evaluate_game(self):
        # #region debug-point D:smoother-game
        self._report_debug(
            "D",
            "smoother_node.py:re_evaluate_game",
            f"[DEBUG] smoother reevaluate game for {self.agent_name}",
            {"has_topo_paths": bool(self.current_topo_paths), "mode": self.current_mode, "last_choice": self.my_last_choice},
        )
        # #endregion
        if not self.current_topo_paths:
            self.get_logger().warn(f"USV {self.my_id} 收到的 PRM 路径为空！可能被障碍物完全封死，无法避障！")
            self.publish_fallback_straight_path()
            return

        now_t = self.get_clock().now().nanoseconds * 1e-9
        # 避障阶段：短时间保持当前选择避免抖动，但不再永久锁死
        if self.current_mode == "AVOIDANCE" and self.my_last_choice is not None and now_t < self.avoidance_lock_until:
            idx = int(self.my_last_choice)
            if 0 <= idx < len(self.current_topo_paths):
                self.publish_path(self.current_topo_paths[idx].path)
            return

        self.game_round += 1
        
        # 如果达到最大轮次，保持当前最优选择一小段时间，然后继续允许重评估
        if self.game_round > self.MAX_GAME_ROUNDS:
            best_choice, _ = self.compute_best_choice()
            if best_choice >= 0:
                self.my_last_choice = best_choice
                self.publish_current_decision(best_choice)
                self.publish_path(self.current_topo_paths[best_choice].path)
                self.avoidance_lock_until = now_t + self.avoidance_decision_hold_sec
            self.game_round = 0
            return
            
        if self.current_mode == "AVOIDANCE" and self.emergency_lock_needed():
            self.emergency_lock_choice()
            return

        best_choice, _ = self.compute_best_choice()
        if best_choice < 0:
            return

        choice_changed = self.my_last_choice != best_choice

        # 只有在决策改变，或者是第一次做决策时，才向邻居广播决策消息
        if choice_changed:
            self.my_last_choice = best_choice
            self.choice_stability_count = 1
            self.get_logger().info(f"*** 非合作决策更新 ***: USV {self.my_id} 决定走路径 {best_choice}")
            self.publish_current_decision(best_choice)
            if self.current_mode == "AVOIDANCE":
                hold_scale = 1.0 if self.last_discussion_neighbors else 0.6
                self.avoidance_lock_until = now_t + self.avoidance_decision_hold_sec * hold_scale
        elif self.my_last_choice is not None:
            self.choice_stability_count += 1

        if self.my_last_choice is not None and self.current_mode == "AVOIDANCE":
            self.publish_path(self.current_topo_paths[self.my_last_choice].path)

        if self.current_mode == "AVOIDANCE":
            recent_candidate_change = (
                self.last_candidate_change_time > 0.0 and
                (now_t - self.last_candidate_change_time) < self.candidate_freeze_sec
            )
            if not recent_candidate_change and self.choice_stability_count >= self.route_lock_repeat_count:
                self.avoidance_lock_until = now_t + self.avoidance_decision_hold_sec

    def finalize_game(self):
        self.cancel_lock_timer()
        self.lock_current_choice("达到收敛条件，发布锁定路径")
    
def main(args=None):
        rclpy.init(args=args)
        node = SmootherNode("smoother_node")
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
if __name__=="__main__":
        main()
