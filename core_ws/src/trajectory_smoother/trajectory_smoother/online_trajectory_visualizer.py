from functools import partial
from typing import Dict, List, Optional, Tuple

from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy
import json
import urllib.request
from std_msgs.msg import String
from topology_interfaces.msg import Decision, TopologicalPathArray
from visualization_msgs.msg import Marker, MarkerArray


def _signature_key(signature: List[float]) -> Tuple[float, ...]:
    return tuple(round(float(v), 6) for v in signature)


class OnlineTrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__("online_trajectory_visualizer")

        self.declare_parameter("agent_id", 1)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("stale_timeout_sec", 3.0)
        self.declare_parameter("selected_hold_sec", 2.0)
        self.declare_parameter("candidate_hold_sec", 1.2)
        self.declare_parameter("static_obstacle_x", [0.0])
        self.declare_parameter("static_obstacle_y", [0.0])
        self.declare_parameter("static_obstacle_radii", [0.0])
        self.declare_parameter("dynamic_obstacle_topics", ["/vessel_e1/odom"])
        self.declare_parameter("dynamic_obstacle_names", ["vessel_e1"])
        self.declare_parameter("dynamic_obstacle_radii", [3.0])

        self.agent_id = int(self.get_parameter("agent_id").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        self.selected_hold_sec = float(self.get_parameter("selected_hold_sec").value)
        self.candidate_hold_sec = float(self.get_parameter("candidate_hold_sec").value)
        self.static_obstacles = list(zip(
            list(self.get_parameter("static_obstacle_x").value),
            list(self.get_parameter("static_obstacle_y").value),
            list(self.get_parameter("static_obstacle_radii").value),
        ))
        self.dynamic_obstacle_topics = list(self.get_parameter("dynamic_obstacle_topics").value)
        self.dynamic_obstacle_names = list(self.get_parameter("dynamic_obstacle_names").value)
        self.dynamic_obstacle_radii = list(self.get_parameter("dynamic_obstacle_radii").value)
        self.agent_name = f"usv_{self.agent_id}"
        self._debug_url, self._debug_session = self._load_debug_env()

        self.colors: Dict[int, Tuple[float, float, float]] = {
            1: (0.12, 0.47, 0.71),
            2: (0.17, 0.63, 0.17),
            3: (0.84, 0.15, 0.16),
            4: (0.58, 0.40, 0.74),
        }
        self.color = self.colors.get(self.agent_id, (0.20, 0.20, 0.80))

        self.current_mode = "FORMATION"
        self.current_position = None
        self.latest_paths: Optional[TopologicalPathArray] = None
        self.latest_paths_time = None
        self.latest_decision: Optional[Decision] = None
        self.latest_decision_time = None
        self.selected_signature: Optional[Tuple[float, ...]] = None
        self.selected_hold_until = None
        self.candidate_hold_until = None
        self.dynamic_obstacles = {}
        self.last_goal_point = None
        self.latest_reference_path: Optional[Path] = None

        self.marker_pub = self.create_publisher(MarkerArray, "trajectory_markers", 10)
        self.create_subscription(TopologicalPathArray, "topological_path", self.paths_callback, 10)
        self.create_subscription(Decision, "topology_decision", self.decision_callback, 10)
        self.create_subscription(String, f"/{self.agent_name}/planner_mode", self.mode_callback, 10)
        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(Path, "reference_path", self.reference_path_callback, 10)
        self.create_subscription(Path, "smooth_trajectory", self.smooth_trajectory_callback, 10)
        obstacle_count = min(
            len(self.dynamic_obstacle_topics),
            len(self.dynamic_obstacle_names),
            len(self.dynamic_obstacle_radii),
        )
        for idx in range(obstacle_count):
            name = str(self.dynamic_obstacle_names[idx])
            topic = str(self.dynamic_obstacle_topics[idx])
            radius = float(self.dynamic_obstacle_radii[idx])
            self.dynamic_obstacles[name] = {
                "position": None,
                "velocity": None,
                "radius": radius,
            }
            self.create_subscription(Odometry, topic, partial(self.dynamic_obstacle_callback, obstacle_name=name), 10)

        self.timer = self.create_timer(1.0 / publish_rate, self.on_timer)
        self.get_logger().info(f"Online trajectory visualizer ready for {self.agent_name}")

    def _load_debug_env(self):
        debug_url = "http://127.0.0.1:7777/event"
        debug_session = "ros-prm-visualization"
        env_path = "/home/lu/paper2/.dbg/ros-prm-visualization.env"
        try:
            with open(env_path, "r", encoding="utf-8") as env_file:
                for line in env_file:
                    line = line.strip()
                    if line.startswith("DEBUG_SERVER_URL="):
                        debug_url = line.split("=", 1)[1]
                    elif line.startswith("DEBUG_SESSION_ID="):
                        debug_session = line.split("=", 1)[1]
        except OSError:
            pass
        return debug_url, debug_session

    def _report_debug(self, hypothesis_id: str, location: str, msg: str, data: Dict):
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

    def odom_callback(self, msg: Odometry):
        self.current_position = msg.pose.pose.position

    def reference_path_callback(self, msg: Path):
        self.latest_reference_path = msg

    def smooth_trajectory_callback(self, msg: Path):
        if msg.poses:
            self.last_goal_point = msg.poses[-1].pose.position

    def dynamic_obstacle_callback(self, msg: Odometry, obstacle_name: str):
        if obstacle_name not in self.dynamic_obstacles:
            return
        self.dynamic_obstacles[obstacle_name]["position"] = msg.pose.pose.position
        self.dynamic_obstacles[obstacle_name]["velocity"] = msg.twist.twist.linear

    def mode_callback(self, msg: String):
        self.current_mode = msg.data

    def paths_callback(self, msg: TopologicalPathArray):
        self.latest_paths = msg
        self.latest_paths_time = self.get_clock().now()
        # #region debug-point A:viz-paths
        self._report_debug(
            "A",
            "online_trajectory_visualizer.py:paths_callback",
            f"[DEBUG] visualizer received paths for {self.agent_name}",
            {"path_count": len(msg.topological_paths), "frame_id": msg.header.frame_id},
        )
        # #endregion

    def decision_callback(self, msg: Decision):
        self.latest_decision = msg
        self.latest_decision_time = self.get_clock().now()
        self.selected_signature = _signature_key(list(msg.h_signature)) if msg.h_signature else None
        self.selected_hold_until = self.get_clock().now() + Duration(seconds=self.selected_hold_sec)
        # #region debug-point A:viz-decision
        self._report_debug(
            "A",
            "online_trajectory_visualizer.py:decision_callback",
            f"[DEBUG] visualizer received decision for {self.agent_name}",
            {"has_signature": bool(msg.h_signature), "locked": bool(msg.is_locked)},
        )
        # #endregion

    def _is_fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9 <= self.stale_timeout

    def _marker(self, marker_id: int, ns: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _path_points(self, path: Path, z: float) -> List:
        points = []
        for pose in path.poses:
            pt = pose.pose.position
            point = type(pt)()
            point.x = pt.x
            point.y = pt.y
            point.z = z
            points.append(point)
        return points

    def _selected_index(self) -> Optional[int]:
        if self.latest_paths is None or self.selected_signature is None:
            return None
        for idx, topo_path in enumerate(self.latest_paths.topological_paths):
            if _signature_key(list(topo_path.h_signature)) == self.selected_signature:
                return idx
        return None

    def _should_show_only_selected(self) -> bool:
        if self.latest_decision is not None and self.latest_decision.is_locked:
            return True
        return False

    def _append_candidate_marker(self, markers: MarkerArray, topo_path: Path, path_idx: int, is_selected: bool):
        line = self._marker(path_idx, "candidate_paths", Marker.LINE_STRIP)
        line.scale.x = 0.24 if is_selected else 0.11
        line.color.r, line.color.g, line.color.b = self.color
        line.color.a = 0.98 if is_selected else 0.42
        line.points = self._path_points(topo_path, 0.18 if is_selected else 0.12)
        markers.markers.append(line)

        if len(line.points) >= 2:
            fan = self._marker(300 + path_idx, "topology_candidates", Marker.LINE_STRIP)
            fan.scale.x = 0.06 if not is_selected else 0.10
            fan.color.r, fan.color.g, fan.color.b = self.color
            fan.color.a = 0.32 if not is_selected else 0.55
            if self.current_position is not None:
                start = type(self.current_position)()
                start.x = float(self.current_position.x)
                start.y = float(self.current_position.y)
                start.z = 0.08
                fan.points.append(start)
            stride = max(1, len(line.points) // 8)
            for idx in range(0, len(line.points), stride):
                fan.points.append(line.points[idx])
            if fan.points[-1] is not line.points[-1]:
                fan.points.append(line.points[-1])
            markers.markers.append(fan)

        if not is_selected and len(line.points) >= 2:
            dashed = self._marker(100 + path_idx, "candidate_dashes", Marker.LINE_LIST)
            dashed.scale.x = 0.09
            dashed.color.r, dashed.color.g, dashed.color.b = self.color
            dashed.color.a = 0.45
            for idx in range(0, len(line.points) - 1, 3):
                dashed.points.append(line.points[idx])
                dashed.points.append(line.points[idx + 1])
            markers.markers.append(dashed)

        if topo_path.poses:
            text = self._marker(200 + path_idx, "candidate_labels", Marker.TEXT_VIEW_FACING)
            text.pose.position = topo_path.poses[-1].pose.position
            text.pose.position.z = 0.9
            text.scale.z = 0.65
            text.color.r, text.color.g, text.color.b = self.color
            text.color.a = 0.95 if is_selected else 0.60
            text.text = f"T{path_idx}"
            markers.markers.append(text)

    def _append_identity_marker(self, markers: MarkerArray):
        if self.current_position is None:
            return
        text = self._marker(800 + self.agent_id, "usv_identity", Marker.TEXT_VIEW_FACING)
        text.pose.position.x = self.current_position.x
        text.pose.position.y = self.current_position.y
        text.pose.position.z = 2.2
        text.scale.z = 1.0
        text.color.r, text.color.g, text.color.b = self.color
        text.color.a = 0.95
        text.text = f"USV-{self.agent_id} | {self.current_mode}"
        markers.markers.append(text)

    def _append_reference_path_marker(self, markers: MarkerArray):
        if self.latest_reference_path is None or not self.latest_reference_path.poses:
            return
        ref_line = self._marker(700 + self.agent_id, "reference_path", Marker.LINE_STRIP)
        ref_line.scale.x = 0.10
        ref_line.color.r, ref_line.color.g, ref_line.color.b = self.color
        ref_line.color.a = 0.28
        ref_line.points = self._path_points(self.latest_reference_path, 0.08)
        markers.markers.append(ref_line)

    def _append_static_obstacle_markers(self, markers: MarkerArray):
        for idx, (x, y, radius) in enumerate(self.static_obstacles):
            radius = float(radius)
            if radius <= 0.0:
                continue
            island = self._marker(900 + idx, "static_islands", Marker.CUBE)
            island.pose.position.x = float(x)
            island.pose.position.y = float(y)
            island.pose.position.z = -0.04
            island.scale.x = 2.6 * radius
            island.scale.y = 1.5 * radius
            island.scale.z = 0.12
            island.color.r = 0.18
            island.color.g = 0.18
            island.color.b = 0.18
            island.color.a = 0.72
            markers.markers.append(island)

            text = self._marker(1000 + idx, "static_island_labels", Marker.TEXT_VIEW_FACING)
            text.pose.position.x = float(x)
            text.pose.position.y = float(y)
            text.pose.position.z = 0.9
            text.scale.z = 0.65
            text.color.r = 0.20
            text.color.g = 0.18
            text.color.b = 0.15
            text.color.a = 0.85
            text.text = f"Island {idx + 1}"
            markers.markers.append(text)

    def _append_start_goal_markers(self, markers: MarkerArray):
        if self.current_position is not None:
            start = self._marker(1100 + self.agent_id, "planner_start", Marker.SPHERE)
            start.pose.position.x = float(self.current_position.x)
            start.pose.position.y = float(self.current_position.y)
            start.pose.position.z = 0.35
            start.scale.x = 0.9
            start.scale.y = 0.9
            start.scale.z = 0.9
            start.color.r, start.color.g, start.color.b = self.color
            start.color.a = 0.95
            markers.markers.append(start)

        if self.last_goal_point is not None:
            goal = self._marker(1110 + self.agent_id, "planner_goal", Marker.SPHERE)
            goal.pose.position.x = float(self.last_goal_point.x)
            goal.pose.position.y = float(self.last_goal_point.y)
            goal.pose.position.z = 0.45
            goal.scale.x = 1.1
            goal.scale.y = 1.1
            goal.scale.z = 1.1
            goal.color.r = 0.98
            goal.color.g = 0.56
            goal.color.b = 0.04
            goal.color.a = 0.98
            markers.markers.append(goal)

            goal_text = self._marker(1120 + self.agent_id, "planner_goal_labels", Marker.TEXT_VIEW_FACING)
            goal_text.pose.position.x = float(self.last_goal_point.x)
            goal_text.pose.position.y = float(self.last_goal_point.y)
            goal_text.pose.position.z = 1.4
            goal_text.scale.z = 0.7
            goal_text.color.r = 0.98
            goal_text.color.g = 0.56
            goal_text.color.b = 0.04
            goal_text.color.a = 0.95
            goal_text.text = f"Goal {self.agent_name}"
            markers.markers.append(goal_text)

    def _append_dynamic_obstacle_markers(self, markers: MarkerArray):
        for idx, (name, track) in enumerate(self.dynamic_obstacles.items()):
            position = track.get("position")
            velocity = track.get("velocity")
            if position is None:
                continue

            label = self._marker(1300 + idx, "dynamic_obstacle_labels", Marker.TEXT_VIEW_FACING)
            label.pose.position.x = float(position.x)
            label.pose.position.y = float(position.y)
            label.pose.position.z = 1.8
            label.scale.z = 0.65
            label.color.r = 0.05
            label.color.g = 0.40
            label.color.b = 0.18
            label.color.a = 0.95
            label.text = name
            markers.markers.append(label)

            if velocity is not None:
                arrow = self._marker(1400 + idx, "dynamic_obstacle_velocity", Marker.ARROW)
                arrow.scale.x = 0.22
                arrow.scale.y = 0.45
                arrow.scale.z = 0.55
                arrow.color.r = 0.05
                arrow.color.g = 0.40
                arrow.color.b = 0.18
                arrow.color.a = 0.90
                start = type(position)()
                start.x = float(position.x)
                start.y = float(position.y)
                start.z = 0.9
                end = type(position)()
                end.x = float(position.x + velocity.x * 4.0)
                end.y = float(position.y + velocity.y * 4.0)
                end.z = 0.9
                arrow.points = [start, end]
                markers.markers.append(arrow)

                prediction = self._marker(1500 + idx, "dynamic_obstacle_predictions", Marker.LINE_STRIP)
                prediction.scale.x = 0.12
                prediction.color.r = 0.05
                prediction.color.g = 0.40
                prediction.color.b = 0.18
                prediction.color.a = 0.75
                for step in range(6):
                    point = type(position)()
                    point.x = float(position.x + velocity.x * step * 1.5)
                    point.y = float(position.y + velocity.y * step * 1.5)
                    point.z = 0.95 + 0.03 * step
                    prediction.points.append(point)
                markers.markers.append(prediction)

    def on_timer(self):
        markers = MarkerArray()
        self._append_identity_marker(markers)
        self._append_reference_path_marker(markers)
        self._append_static_obstacle_markers(markers)
        self._append_start_goal_markers(markers)
        self._append_dynamic_obstacle_markers(markers)

        paths_fresh = self._is_fresh(self.latest_paths_time)
        if self.latest_paths is None:
            # #region debug-point A:viz-empty
            self._report_debug(
                "A",
                "online_trajectory_visualizer.py:on_timer",
                f"[DEBUG] visualizer publish empty markers for {self.agent_name}",
                {"reason": "missing_paths", "mode": self.current_mode},
            )
            # #endregion
            self.marker_pub.publish(markers)
            return

        topo_paths = list(self.latest_paths.topological_paths)
        if not topo_paths:
            self.marker_pub.publish(markers)
            return

        selected_idx = self._selected_index()
        show_only_selected = self._should_show_only_selected()
        now = self.get_clock().now()

        if len(topo_paths) <= 1:
            show_only_selected = False
            if selected_idx is None:
                selected_idx = 0

        if self.current_mode != "AVOIDANCE" and self.latest_decision is None:
            show_only_selected = False

        if self.latest_decision is not None and self.latest_decision.is_locked:
            show_only_selected = True

        if self.candidate_hold_until is not None and now <= self.candidate_hold_until:
            show_only_selected = False

        if self.selected_hold_until is not None and now > self.selected_hold_until:
            self.selected_signature = None
            show_only_selected = False

        for idx, topo_path in enumerate(topo_paths):
            is_selected = selected_idx == idx
            if show_only_selected and not is_selected:
                continue
            self._append_candidate_marker(markers, topo_path.path, idx, is_selected)

        # #region debug-point A:viz-publish
        self._report_debug(
            "A",
            "online_trajectory_visualizer.py:on_timer",
            f"[DEBUG] visualizer published markers for {self.agent_name}",
            {
                "mode": self.current_mode,
                "topo_count": len(topo_paths),
                "selected_idx": selected_idx,
                "show_only_selected": show_only_selected,
                "marker_count": len(markers.markers),
                "frame_id": self.frame_id,
                "paths_fresh": paths_fresh,
            },
        )
        # #endregion

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = OnlineTrajectoryVisualizer()
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
