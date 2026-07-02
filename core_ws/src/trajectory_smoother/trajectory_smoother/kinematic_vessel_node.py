import json
import math
import urllib.request

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class KinematicVesselNode(Node):
    def __init__(self):
        super().__init__("kinematic_vessel_node")

        self.declare_parameter("world_name", "dave_ocean_waves_fixed")
        self.declare_parameter("model_name", "vessel_e")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("update_rate", 10.0)
        self.declare_parameter("initial_pose", [0.0, 0.0, 0.2, 0.0])
        self.declare_parameter("linear_velocity", [0.5, 0.0])
        self.declare_parameter("target", [135.0, 30.0, 0.0])
        self.declare_parameter("static_obstacle_x", [0.0])
        self.declare_parameter("static_obstacle_y", [0.0])
        self.declare_parameter("static_obstacle_radii", [0.0])
        self.declare_parameter("vessel_nominal_speed", 0.95)
        self.declare_parameter("vessel_turn_rate_limit", 0.38)
        self.declare_parameter("vessel_heading_kp", 1.4)
        self.declare_parameter("vessel_heading_deadband_deg", 4.0)
        self.declare_parameter("vessel_path_lookahead", 10.0)
        self.declare_parameter("vessel_path_lane_half_width", 7.0)
        self.declare_parameter("vessel_path_reacquire_gain", 0.18)
        self.declare_parameter("vessel_collision_horizon_sec", 22.0)
        self.declare_parameter("vessel_collision_buffer", 2.0)
        self.declare_parameter("vessel_emergency_brake_distance", 5.0)
        self.declare_parameter("vessel_avoidance_offset", 10.0)
        self.declare_parameter("vessel_safe_clearance_margin", 2.5)
        self.declare_parameter("vessel_goal_tolerance", 2.0)

        self.world_name = str(self.get_parameter("world_name").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.update_rate = max(1.0, float(self.get_parameter("update_rate").value))

        initial_pose = list(self.get_parameter("initial_pose").value)
        linear_velocity = list(self.get_parameter("linear_velocity").value)
        target = list(self.get_parameter("target").value)
        if len(initial_pose) < 4:
            initial_pose = [0.0, 0.0, 0.2, 0.0]
        if len(linear_velocity) < 2:
            linear_velocity = [0.5, 0.0]
        if len(target) < 2:
            target = [135.0, 30.0, 0.0]

        self.x = float(initial_pose[0])
        self.y = float(initial_pose[1])
        self.z = float(initial_pose[2])
        self.yaw = float(initial_pose[3])
        self.initial_velocity = np.array([float(linear_velocity[0]), float(linear_velocity[1])], dtype=float)
        self.goal = np.array([float(target[0]), float(target[1])], dtype=float)
        self.start = np.array([float(initial_pose[0]), float(initial_pose[1])], dtype=float)
        self.patrol_target_a = self.goal.copy()
        self.patrol_target_b = self.start.copy()
        self.patrol_active = True  # 往返巡逻，不停车

        self.nominal_speed = max(0.2, float(self.get_parameter("vessel_nominal_speed").value))
        self.turn_rate_limit = max(0.05, float(self.get_parameter("vessel_turn_rate_limit").value))
        self.heading_kp = max(0.1, float(self.get_parameter("vessel_heading_kp").value))
        self.heading_deadband = math.radians(float(self.get_parameter("vessel_heading_deadband_deg").value))
        self.lookahead = max(2.0, float(self.get_parameter("vessel_path_lookahead").value))
        self.path_lane_half_width = max(2.0, float(self.get_parameter("vessel_path_lane_half_width").value))
        self.path_reacquire_gain = max(0.01, float(self.get_parameter("vessel_path_reacquire_gain").value))
        self.collision_horizon_sec = max(2.0, float(self.get_parameter("vessel_collision_horizon_sec").value))
        self.collision_buffer = max(0.0, float(self.get_parameter("vessel_collision_buffer").value))
        self.emergency_brake_distance = max(0.5, float(self.get_parameter("vessel_emergency_brake_distance").value))
        self.avoidance_offset = max(1.0, float(self.get_parameter("vessel_avoidance_offset").value))
        self.safe_clearance_margin = max(0.0, float(self.get_parameter("vessel_safe_clearance_margin").value))
        self.goal_tolerance = max(0.5, float(self.get_parameter("vessel_goal_tolerance").value))

        obstacle_x = list(self.get_parameter("static_obstacle_x").value)
        obstacle_y = list(self.get_parameter("static_obstacle_y").value)
        obstacle_r = list(self.get_parameter("static_obstacle_radii").value)
        self.static_obstacles = [
            np.array([float(x), float(y), float(r)], dtype=float)
            for x, y, r in zip(obstacle_x, obstacle_y, obstacle_r)
            if float(r) > 0.0
        ]

        self._debug_url, self._debug_session = self._load_debug_env()
        self.current_speed = max(self.nominal_speed, float(np.linalg.norm(self.initial_velocity)))
        self.vx = self.current_speed * math.cos(self.yaw)
        self.vy = self.current_speed * math.sin(self.yaw)
        self.last_avoidance_side = 1.0
        self.current_mode = "CRUISE"
        self.current_target = self.goal.copy()

        self.entity = Entity()
        self.entity.name = self.model_name

        service_name = f"/world/{self.world_name}/set_pose"
        self.set_pose_client = self.create_client(SetEntityPose, service_name)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.pending_future = None
        self.last_update_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / self.update_rate, self.on_timer)
        self.get_logger().info(
            f"Kinematic vessel ready: model={self.model_name}, world={self.world_name}, "
            f"pose=({self.x:.1f}, {self.y:.1f}, {self.z:.1f}, yaw={self.yaw:.2f}), goal=({self.goal[0]:.1f}, {self.goal[1]:.1f})"
        )

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

    def _report_debug(self, hypothesis_id, location, msg, data, run_id="post-fix"):
        payload = {
            "sessionId": self._debug_session,
            "runId": run_id,
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

    def _position(self):
        return np.array([self.x, self.y], dtype=float)

    def _goal_vector(self):
        return self.goal - self._position()

    def _distance_to_goal(self):
        return float(np.linalg.norm(self._goal_vector()))

    def _path_tracking_target(self):
        start = np.array([self.x, self.y], dtype=float)
        goal_dir = self._goal_vector()
        norm = float(np.linalg.norm(goal_dir))
        if norm < 1e-6:
            return self.goal.copy(), 0.0
        direction = goal_dir / norm
        progress = min(self.lookahead, norm)
        track_target = start + direction * progress
        cross_track = direction[0] * (self.y - start[1]) - direction[1] * (self.x - start[0])
        return track_target - np.array([-direction[1], direction[0]]) * cross_track * self.path_reacquire_gain, cross_track

    def _line_obstacle_intersection(self, obstacle):
        position = self._position()
        to_goal = self.goal - position
        dist_goal = float(np.linalg.norm(to_goal))
        if dist_goal < 1e-6:
            return None
        direction = to_goal / dist_goal
        rel = obstacle[:2] - position
        forward = float(np.dot(rel, direction))
        if forward < 0.0 or forward > self.collision_horizon_sec * self.nominal_speed:
            return None
        lateral = abs(float(direction[0] * rel[1] - direction[1] * rel[0]))
        inflated = float(obstacle[2] + self.collision_buffer + self.safe_clearance_margin)
        if lateral > inflated:
            return None
        clearance = max(forward - math.sqrt(max(inflated ** 2 - lateral ** 2, 0.0)), 0.0)
        return {
            "forward": forward,
            "lateral": lateral,
            "inflated_radius": inflated,
            "clearance": clearance,
            "center": obstacle[:2],
        }

    def _nearest_hazard(self):
        hazards = []
        for obstacle in self.static_obstacles:
            info = self._line_obstacle_intersection(obstacle)
            if info is None:
                continue
            hazards.append((info["clearance"], info))
        if not hazards:
            return None
        hazards.sort(key=lambda item: item[0])
        return hazards[0][1]

    def _avoidance_target(self, hazard):
        position = self._position()
        direction = self._goal_vector()
        norm = float(np.linalg.norm(direction))
        if norm < 1e-6:
            direction = np.array([math.cos(self.yaw), math.sin(self.yaw)], dtype=float)
        else:
            direction = direction / norm
        lateral = np.array([-direction[1], direction[0]], dtype=float)
        rel = hazard["center"] - position
        signed = float(direction[0] * rel[1] - direction[1] * rel[0])
        side = -1.0 if signed >= 0.0 else 1.0
        self.last_avoidance_side = side
        tangent_point = hazard["center"] + lateral * side * (hazard["inflated_radius"] + self.avoidance_offset)
        forward_target = tangent_point + direction * self.lookahead
        return forward_target

    def _compute_guidance(self):
        distance_to_goal = self._distance_to_goal()
        if distance_to_goal <= self.goal_tolerance and self.patrol_active:
            # 往返巡逻：到达目标后交换 AB 点，不停车
            self.patrol_target_a, self.patrol_target_b = self.patrol_target_b, self.patrol_target_a
            self.goal = self.patrol_target_a.copy()
            distance_to_goal = self._distance_to_goal()

        track_target, cross_track_error = self._path_tracking_target()
        hazard = self._nearest_hazard()
        if hazard is not None:
            self.current_mode = "EMERGENCY_BRAKE" if hazard["clearance"] <= self.emergency_brake_distance else "AVOID_STATIC"
            if self.current_mode == "AVOID_STATIC":
                self.current_target = self._avoidance_target(hazard)
            else:
                self.current_target = self._position()
        else:
            self.current_mode = "CRUISE" if abs(cross_track_error) <= self.path_lane_half_width else "REACQUIRE_PATH"
            self.current_target = track_target

        desired_heading = math.atan2(self.current_target[1] - self.y, self.current_target[0] - self.x)
        heading_error = wrap_angle(desired_heading - self.yaw)
        desired_speed = self.nominal_speed
        if self.current_mode == "EMERGENCY_BRAKE":
            desired_speed = 0.0
        elif self.current_mode == "AVOID_STATIC":
            desired_speed = max(0.35, 0.7 * self.nominal_speed)
        elif abs(cross_track_error) > self.path_lane_half_width:
            desired_speed = max(0.45, 0.8 * self.nominal_speed)

        return desired_speed, desired_heading, hazard, cross_track_error

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        dt = max(0.0, min(dt, 0.2))

        desired_speed, desired_heading, hazard, cross_track_error = self._compute_guidance()
        heading_error = wrap_angle(desired_heading - self.yaw)
        yaw_step = max(-self.turn_rate_limit * dt, min(self.turn_rate_limit * dt, heading_error))
        self.yaw = wrap_angle(self.yaw + yaw_step)
        speed_step = max(-0.8 * dt, min(0.8 * dt, desired_speed - self.current_speed))
        self.current_speed = max(0.0, self.current_speed + speed_step)
        self.vx = self.current_speed * math.cos(self.yaw)
        self.vy = self.current_speed * math.sin(self.yaw)
        self.x += self.vx * dt
        self.y += self.vy * dt

        # #region debug-point C:vessel-guidance
        self._report_debug(
            "C",
            "kinematic_vessel_node.py:on_timer",
            f"[DEBUG] vessel guidance update for {self.model_name}",
            {
                "mode": self.current_mode,
                "x": self.x,
                "y": self.y,
                "yaw": self.yaw,
                "speed": self.current_speed,
                "desired_heading": desired_heading,
                "cross_track_error": cross_track_error,
                "hazard_clearance": None if hazard is None else hazard["clearance"],
            },
        )
        # #endregion

        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z
        pose.orientation = quaternion_from_yaw(self.yaw)

        self.publish_odom(now, pose)

        if not self.set_pose_client.service_is_ready():
            return
        if self.pending_future is not None and not self.pending_future.done():
            return

        request = SetEntityPose.Request()
        request.entity = self.entity
        request.pose = pose
        self.pending_future = self.set_pose_client.call_async(request)

    def publish_odom(self, now, pose):
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose = pose
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        self.odom_pub.publish(msg)
        # #region debug-point C:vessel-odom
        self._report_debug(
            "C",
            "kinematic_vessel_node.py:publish_odom",
            f"[DEBUG] vessel published odom for {self.model_name}",
            {"x": pose.position.x, "y": pose.position.y, "yaw": self.yaw, "mode": self.current_mode},
        )
        # #endregion


def main(args=None):
    rclpy.init(args=args)
    node = KinematicVesselNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
