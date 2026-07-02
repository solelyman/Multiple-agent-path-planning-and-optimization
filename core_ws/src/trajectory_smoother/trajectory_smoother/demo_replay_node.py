import csv
import math
from collections import defaultdict
from pathlib import Path

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def _point(x: float, y: float, z: float = 0.0) -> Point:
    pt = Point()
    pt.x = float(x)
    pt.y = float(y)
    pt.z = float(z)
    return pt


def _yaw_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


class DemoReplayNode(Node):
    def __init__(self):
        super().__init__("demo_replay_node")

        default_trace = "/home/lu/paper2/results/final_fig_data/trace_demo_ours_seed1035.csv"
        default_obstacles = "/home/lu/paper2/results/final_fig_data/obstacles_demo_ours_seed1035.csv"
        self.declare_parameter("trace_csv", default_trace)
        self.declare_parameter("obstacle_csv", default_obstacles)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("timer_period", 0.12)
        self.declare_parameter("step_stride", 2)
        self.declare_parameter("tail_length", 42)
        self.declare_parameter("loop", True)
        self.declare_parameter("show_candidate_lanes", True)
        self.declare_parameter("use_robot_model", True)
        self.declare_parameter("usv_mesh", "file:///home/lu/paper2/core_ws/src/model/mesh/heron/heron_base.stl")
        self.declare_parameter("vessel_e_mesh", "file:///home/lu/paper2/core_ws/src/model/models/vessel_e/meshes/Boat05.dae")
        self.declare_parameter("vessel_f_mesh", "file:///home/lu/paper2/core_ws/src/model/models/vessel_f/meshes/Boat06.dae")

        self.frame_id = self.get_parameter("frame_id").value
        self.timer_period = float(self.get_parameter("timer_period").value)
        self.step_stride = int(self.get_parameter("step_stride").value)
        self.tail_length = int(self.get_parameter("tail_length").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.show_candidate_lanes = bool(self.get_parameter("show_candidate_lanes").value)
        self.use_robot_model = bool(self.get_parameter("use_robot_model").value)
        self.candidate_display_obstacle_dist = 34.0
        self.candidate_display_agent_dist = 14.0
        self.max_yaw_rate = 0.35
        self.smoothed_yaw = {}
        self.prev_step = None
        self.candidate_display_steps = 10
        self.selected_display_steps = 16
        self.decision_rearm_steps = 10
        self.decision_state = {}
        self.lane_candidates = [-27.0, -18.0, -9.0, 0.0, 9.0, 18.0, 27.0]
        self.x_min = -30.0
        self.x_max = 125.0
        self.y_min = -40.0
        self.y_max = 40.0
        self.usv_mesh = self.get_parameter("usv_mesh").value
        self.vessel_e_mesh = self.get_parameter("vessel_e_mesh").value
        self.vessel_f_mesh = self.get_parameter("vessel_f_mesh").value
        self.usv_yaw_offset = math.pi
        self.vessel_e_yaw_offset = -math.pi / 2.0
        self.vessel_f_yaw_offset = math.pi / 2.0
        trace_csv = Path(self.get_parameter("trace_csv").value)
        obstacle_csv = Path(self.get_parameter("obstacle_csv").value)

        self.agent_traces, self.max_step = self._load_trace(trace_csv)
        self.obstacles_by_step = self._load_obstacles(obstacle_csv)
        self._assign_dynamic_obstacle_models()
        self.agent_ids = sorted(self.agent_traces.keys())
        self.colors = {
            0: (0.12, 0.47, 0.71),
            1: (0.17, 0.63, 0.17),
            2: (0.84, 0.15, 0.16),
            3: (0.58, 0.40, 0.74),
        }

        self.marker_pub = self.create_publisher(MarkerArray, "/demo/scene_markers", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.path_pubs = {
            agent_id: self.create_publisher(PathMsg, f"/demo/agent_{agent_id + 1}/path", 10)
            for agent_id in self.agent_ids
        }

        self.current_frame_index = 0
        self.timer = self.create_timer(self.timer_period, self.on_timer)
        self.get_logger().info(f"Loaded demo replay trace from {trace_csv}")

    def _load_trace(self, trace_csv: Path):
        by_agent = defaultdict(list)
        max_step = 0
        with trace_csv.open() as f:
            reader = csv.DictReader(f)
            for row in reader:
                step = int(float(row["step"]))
                agent_id = int(row["agent_id"])
                by_agent[agent_id].append(
                    {
                        "step": step,
                        "time": float(row["time"]),
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                        "vx": float(row["vx"]),
                        "psi": float(row.get("psi", 0.0)),
                        "lane": float(row.get("lane", row["y"])),
                        "nearest_agent_dist": float(row["nearest_agent_dist"]),
                        "nearest_obstacle_dist": float(row["nearest_obstacle_dist"]),
                    }
                )
                max_step = max(max_step, step)
        return by_agent, max_step

    def _load_obstacles(self, obstacle_csv: Path):
        by_step = defaultdict(list)
        with obstacle_csv.open() as f:
            reader = csv.DictReader(f)
            for row in reader:
                by_step[int(float(row["step"]))].append(
                    {
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                        "radius": float(row["radius"]),
                        "kind": row.get("kind", "dynamic"),
                        "obstacle_id": int(float(row.get("obstacle_id", 0))),
                    }
                )
        return by_step

    def _assign_dynamic_obstacle_models(self):
        self.dynamic_model_by_id = {}
        dynamic_ids = sorted(
            {
                obs["obstacle_id"]
                for obs_list in self.obstacles_by_step.values()
                for obs in obs_list
                if obs.get("kind", "dynamic") != "static"
            }
        )
        for obstacle_id in dynamic_ids:
            self.dynamic_model_by_id[obstacle_id] = self.vessel_e_mesh if obstacle_id % 2 == 0 else self.vessel_f_mesh

    def _state_at(self, agent_id: int, step: int):
        trace = self.agent_traces[agent_id]
        idx = min(step, len(trace) - 1)
        return trace[idx]

    def _decision_state_for(self, agent_id: int):
        if agent_id not in self.decision_state:
            self.decision_state[agent_id] = {
                "last_active": False,
                "last_event_step": -10_000,
                "candidate_until": -1,
                "selected_until": -1,
                "selected_lane": 0.0,
            }
        return self.decision_state[agent_id]

    def _is_decision_active(self, agent_id: int, step: int, state: dict) -> bool:
        # Trigger local replanning only when there is a relevant object in front / nearby,
        # instead of relying on the global trace minima that can keep the UI active too long.
        for obs in self.obstacles_by_step.get(step, []):
            dx = obs["x"] - state["x"]
            dy = obs["y"] - state["y"]
            distance = math.hypot(dx, dy) - obs["radius"]
            if -6.0 <= dx <= 34.0 and abs(dy) <= 22.0 and distance < self.candidate_display_obstacle_dist:
                return True

        for other_id in self.agent_ids:
            if other_id == agent_id:
                continue
            other = self._state_at(other_id, step)
            dx = other["x"] - state["x"]
            dy = other["y"] - state["y"]
            distance = math.hypot(dx, dy)
            if -6.0 <= dx <= 18.0 and abs(dy) <= 14.0 and distance < self.candidate_display_agent_dist:
                return True

        return False

    def _wrap_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _smoothed_agent_yaw(self, agent_id: int, target_yaw: float, dt: float) -> float:
        if agent_id not in self.smoothed_yaw or dt <= 0.0:
            self.smoothed_yaw[agent_id] = target_yaw
            return target_yaw
        current = self.smoothed_yaw[agent_id]
        delta = self._wrap_angle(target_yaw - current)
        max_delta = self.max_yaw_rate * dt
        delta = max(-max_delta, min(max_delta, delta))
        current = self._wrap_angle(current + delta)
        self.smoothed_yaw[agent_id] = current
        return current

    def _publish_agent_tf(self, agent_id: int, state: dict, yaw: float):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = f"usv_{agent_id + 1}/base_link"
        transform.transform.translation.x = state["x"]
        transform.transform.translation.y = state["y"]
        transform.transform.translation.z = 0.20
        transform.transform.rotation = _yaw_quaternion(yaw + self.usv_yaw_offset)
        self.tf_broadcaster.sendTransform(transform)

    def _make_marker(self, marker_id: int, ns: str, marker_type: int):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _bezier_points(
        self,
        p0,
        p1,
        p2,
        p3,
        samples: int = 30,
        z: float = 0.08,
    ):
        points = []
        for i in range(samples):
            s = i / float(max(1, samples - 1))
            one_minus = 1.0 - s
            x = (
                one_minus * one_minus * one_minus * p0[0]
                + 3.0 * one_minus * one_minus * s * p1[0]
                + 3.0 * one_minus * s * s * p2[0]
                + s * s * s * p3[0]
            )
            y = (
                one_minus * one_minus * one_minus * p0[1]
                + 3.0 * one_minus * one_minus * s * p1[1]
                + 3.0 * one_minus * s * s * p2[1]
                + s * s * s * p3[1]
            )
            points.append(_point(x, y, z))
        return points

    def _candidate_curve_points(
        self,
        state: dict,
        display_yaw: float,
        lane_y: float,
        length: float = 30.0,
        samples: int = 30,
        z: float = 0.10,
    ):
        speed = max(0.8, state.get("vx", 1.0))
        start = (state["x"], state["y"])
        end = (min(self.x_max, state["x"] + length), lane_y)
        start_tangent = max(5.0, 2.8 * speed)
        end_tangent = max(7.0, 0.32 * length)
        p1 = (
            start[0] + start_tangent * math.cos(display_yaw),
            start[1] + start_tangent * math.sin(display_yaw),
        )
        p2 = (
            end[0] - end_tangent,
            lane_y,
        )
        if p2[0] <= p1[0]:
            p2 = (p1[0] + 2.0, lane_y)
        return self._bezier_points(start, p1, p2, end, samples=samples, z=z)

    def _update_decision_visual_state(self, agent_id: int, step: int, state: dict):
        decision = self._decision_state_for(agent_id)
        active = self._is_decision_active(agent_id, step, state)
        if active and not decision["last_active"] and step - decision["last_event_step"] >= self.decision_rearm_steps:
            decision["last_event_step"] = step
            decision["candidate_until"] = step + self.candidate_display_steps
            decision["selected_until"] = step + self.selected_display_steps
        if active:
            decision["selected_until"] = max(decision["selected_until"], step + 4)
        decision["selected_lane"] = state.get("lane", state["y"])
        decision["last_active"] = active
        if step <= decision["candidate_until"]:
            return "candidates"
        if step <= decision["selected_until"]:
            return "selected"
        return None

    def _add_environment_shell(self, markers: MarkerArray):
        floor = self._make_marker(9000, "scene_shell", Marker.CUBE)
        floor.pose.position = _point((self.x_min + self.x_max) * 0.5, (self.y_min + self.y_max) * 0.5, -0.08)
        floor.scale.x = self.x_max - self.x_min
        floor.scale.y = self.y_max - self.y_min
        floor.scale.z = 0.05
        floor.color.r = 0.97
        floor.color.g = 0.97
        floor.color.b = 0.97
        floor.color.a = 1.0
        markers.markers.append(floor)

        wall_specs = [
            (9001, (self.x_min + self.x_max) * 0.5, self.y_min, 0.75, self.x_max - self.x_min, 0.8),
            (9002, (self.x_min + self.x_max) * 0.5, self.y_max, 0.75, self.x_max - self.x_min, 0.8),
            (9003, self.x_min, (self.y_min + self.y_max) * 0.5, 0.75, self.y_max - self.y_min, 0.8),
            (9004, self.x_max, (self.y_min + self.y_max) * 0.5, 0.75, self.y_max - self.y_min, 0.8),
        ]
        for marker_id, x, y, z, span, thickness in wall_specs:
            wall = self._make_marker(marker_id, "scene_shell", Marker.CUBE)
            wall.pose.position = _point(x, y, z)
            if marker_id in (9001, 9002):
                wall.scale.x = span
                wall.scale.y = thickness
            else:
                wall.scale.x = thickness
                wall.scale.y = span
            wall.scale.z = 1.5
            wall.color.r = 0.24
            wall.color.g = 0.24
            wall.color.b = 0.24
            wall.color.a = 1.0
            markers.markers.append(wall)
    def _add_candidate_curves(self, markers: MarkerArray, agent_id: int, state: dict, display_yaw: float, color):
        selected_lane = state.get("lane", state["y"])
        nearest_lanes = sorted(self.lane_candidates, key=lambda lane: abs(lane - state["y"]))[:3]
        if selected_lane not in nearest_lanes:
            nearest_lanes.append(selected_lane)
        for local_idx, lane_y in enumerate(nearest_lanes):
            is_selected = abs(lane_y - selected_lane) < 1e-3
            marker = self._make_marker(5000 + agent_id * 20 + local_idx, "candidate_curves", Marker.LINE_STRIP)
            marker.scale.x = 0.18 if is_selected else 0.07
            marker.color.r, marker.color.g, marker.color.b = color
            marker.points = self._candidate_curve_points(state, display_yaw, lane_y, length=26.0 if is_selected else 22.0)
            marker.color.a = 0.95 if is_selected else 0.22
            markers.markers.append(marker)

            if not is_selected:
                dash = self._make_marker(6000 + agent_id * 20 + local_idx, "candidate_dashes", Marker.LINE_LIST)
                dash.scale.x = 0.08
                dash.color.r, dash.color.g, dash.color.b = color
                pts = self._candidate_curve_points(state, display_yaw, lane_y, length=22.0, samples=18, z=0.11)
                dash.color.a = 0.35
                for j in range(0, len(pts) - 1, 3):
                    dash.points.append(pts[j])
                    dash.points.append(pts[j + 1])
                markers.markers.append(dash)

    def on_timer(self):
        step = self.current_frame_index * self.step_stride
        if step > self.max_step:
            if self.loop:
                self.current_frame_index = 0
                self.smoothed_yaw.clear()
                self.prev_step = None
                step = 0
            else:
                return

        dt = self.timer_period
        if self.prev_step is not None:
            dt = max(self.timer_period, abs(step - self.prev_step) * 0.2)
        self.prev_step = step

        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        self._add_environment_shell(markers)

        if self.show_candidate_lanes:
            active_any = any(
                self._update_decision_visual_state(agent_id, step, self._state_at(agent_id, step)) == "candidates"
                for agent_id in self.agent_ids
            )
            if active_any:
                for lane_idx, lane_y in enumerate(self.lane_candidates):
                    lane_marker = self._make_marker(10 + lane_idx, "candidate_lanes", Marker.LINE_STRIP)
                    lane_marker.scale.x = 0.08
                    lane_marker.color.r = 0.15
                    lane_marker.color.g = 0.15
                    lane_marker.color.b = 0.15
                    lane_marker.color.a = 0.18
                    lane_marker.points = [_point(self.x_min, lane_y, 0.02), _point(self.x_max, lane_y, 0.02)]
                    markers.markers.append(lane_marker)

        obstacle_id = 0
        for obs in self.obstacles_by_step.get(step, []):
            if obs["kind"] == "static":
                marker = self._make_marker(obstacle_id, "static_obstacles", Marker.CYLINDER)
                marker.pose.position = _point(obs["x"], obs["y"], 0.0)
                marker.scale.x = 2.0 * obs["radius"]
                marker.scale.y = 2.0 * obs["radius"]
                marker.scale.z = 0.4
                marker.color.r = 0.55
                marker.color.g = 0.55
                marker.color.b = 0.55
                marker.color.a = 0.68
                markers.markers.append(marker)
            else:
                safety_marker = self._make_marker(1000 + obstacle_id, "dynamic_safety", Marker.CYLINDER)
                safety_marker.pose.position = _point(obs["x"], obs["y"], 0.0)
                safety_marker.scale.x = 2.0 * obs["radius"]
                safety_marker.scale.y = 2.0 * obs["radius"]
                safety_marker.scale.z = 0.15
                safety_marker.color.r = 0.20
                safety_marker.color.g = 0.20
                safety_marker.color.b = 0.20
                safety_marker.color.a = 0.20
                markers.markers.append(safety_marker)

                mesh_marker = self._make_marker(2000 + obstacle_id, "dynamic_vessels", Marker.MESH_RESOURCE)
                mesh_marker.mesh_resource = self.dynamic_model_by_id.get(obs["obstacle_id"], self.vessel_e_mesh)
                mesh_marker.mesh_use_embedded_materials = True
                mesh_marker.pose.position = _point(obs["x"], obs["y"], 0.25)
                motion_yaw = math.pi if obs["obstacle_id"] % 2 else math.pi / 2.0
                mesh_offset = self.vessel_f_yaw_offset if obs["obstacle_id"] % 2 else self.vessel_e_yaw_offset
                mesh_marker.pose.orientation = _yaw_quaternion(motion_yaw + mesh_offset)
                scale = max(0.85, obs["radius"] * 0.26)
                mesh_marker.scale.x = scale
                mesh_marker.scale.y = scale
                mesh_marker.scale.z = scale
                mesh_marker.color.a = 1.0
                markers.markers.append(mesh_marker)
            obstacle_id += 1

        for agent_id in self.agent_ids:
            state = self._state_at(agent_id, step)
            color = self.colors.get(agent_id, (0.2, 0.2, 0.8))
            display_yaw = self._smoothed_agent_yaw(agent_id, state["psi"], dt)
            self._publish_agent_tf(agent_id, state, display_yaw)
            decision_phase = self._update_decision_visual_state(agent_id, step, state)
            if decision_phase == "candidates":
                self._add_candidate_curves(markers, agent_id, state, display_yaw, color)

            if decision_phase in ("candidates", "selected"):
                selected_marker = self._make_marker(80 + agent_id, "selected_lanes", Marker.LINE_STRIP)
                selected_marker.scale.x = 0.19
                selected_marker.color.r, selected_marker.color.g, selected_marker.color.b = color
                selected_marker.color.a = 0.85 if decision_phase == "selected" else 0.45
                selected_y = self._decision_state_for(agent_id)["selected_lane"]
                selected_marker.points = self._candidate_curve_points(state, display_yaw, selected_y, length=30.0, samples=32, z=0.12)
                markers.markers.append(selected_marker)

            tail_marker = self._make_marker(100 + agent_id, "tails", Marker.LINE_STRIP)
            tail_marker.scale.x = 0.45
            tail_marker.color.r, tail_marker.color.g, tail_marker.color.b = color
            tail_marker.color.a = 0.95

            start_step = max(0, step - self.tail_length)
            for hist_step in range(start_step, step + 1, self.step_stride):
                hist = self._state_at(agent_id, hist_step)
                tail_marker.points.append(_point(hist["x"], hist["y"], 0.1))
            markers.markers.append(tail_marker)

            if not self.use_robot_model:
                body_marker = self._make_marker(200 + agent_id, "agents", Marker.MESH_RESOURCE)
                body_marker.mesh_resource = self.usv_mesh
                body_marker.mesh_use_embedded_materials = False
                body_marker.pose.position = _point(state["x"], state["y"], 0.20)
                body_marker.pose.orientation = _yaw_quaternion(state["psi"] + self.usv_yaw_offset)
                body_marker.scale.x = 0.115
                body_marker.scale.y = 0.115
                body_marker.scale.z = 0.115
                body_marker.color.r, body_marker.color.g, body_marker.color.b = color
                body_marker.color.a = 1.0
                markers.markers.append(body_marker)

            heading_marker = self._make_marker(300 + agent_id, "heading", Marker.LINE_STRIP)
            heading_marker.scale.x = 0.25
            heading_marker.color.r, heading_marker.color.g, heading_marker.color.b = color
            heading_marker.color.a = 1.0
            heading_len = 3.8
            tip_x = state["x"] + heading_len * max(0.3, state["vx"] / 3.4) * math.cos(display_yaw)
            tip_y = state["y"] + heading_len * max(0.3, state["vx"] / 3.4) * math.sin(display_yaw)
            heading_marker.points = [_point(state["x"], state["y"], 0.2), _point(tip_x, tip_y, 0.2)]
            markers.markers.append(heading_marker)

            text_marker = self._make_marker(400 + agent_id, "labels", Marker.TEXT_VIEW_FACING)
            text_marker.pose.position = _point(state["x"], state["y"], 2.0)
            text_marker.scale.z = 1.8
            text_marker.color.r, text_marker.color.g, text_marker.color.b = color
            text_marker.color.a = 1.0
            text_marker.text = f"USV {agent_id + 1}"
            markers.markers.append(text_marker)

            path_msg = PathMsg()
            path_msg.header.frame_id = self.frame_id
            path_msg.header.stamp = self.get_clock().now().to_msg()
            for hist_step in range(start_step, step + 1, self.step_stride):
                hist = self._state_at(agent_id, hist_step)
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = hist["x"]
                pose.pose.position.y = hist["y"]
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            self.path_pubs[agent_id].publish(path_msg)

        self.marker_pub.publish(markers)
        self.current_frame_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = DemoReplayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
