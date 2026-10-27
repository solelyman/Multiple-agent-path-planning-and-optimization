#!/usr/bin/env python3
"""
Lightweight dynamic obstacle vessel simulator.
Each vessel moves from its initial position toward its target in a straight line,
publishing Odometry for the planner to consume.
No Gazebo dependency.
"""
import math
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def _quat_from_yaw(yaw: float) -> Quaternion:
    return _quat_from_rpy(0.0, 0.0, yaw)


def _mesh_orientation(mesh_resource: str, yaw: float) -> Quaternion:
    if "Boat05" in mesh_resource:
        return _quat_from_rpy(0.0, 0.0, yaw - math.pi / 2.0)
    if "Boat06" in mesh_resource:
        return _quat_from_rpy(math.pi / 2.0, 0.0, yaw + math.pi / 2.0)
    return _quat_from_yaw(yaw)


class DynamicObstacleVessel(Node):
    def __init__(self):
        super().__init__("dynamic_obstacle_node")

        self.declare_parameter("vessels", "[]")
        self.declare_parameter("update_rate", 10.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("speed_noise_std", 0.0)
        self.declare_parameter("heading_noise_std", 0.0)
        self.declare_parameter("corridor_y_min", -14.0)
        self.declare_parameter("corridor_y_max", 14.0)
        self.declare_parameter("marker_topic", "/debris_models")
        self.declare_parameter("publish_corridor_walls", True)

        vessels_json = self.get_parameter("vessels").value
        update_rate = self.get_parameter("update_rate").value
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.speed_noise_std = float(self.get_parameter("speed_noise_std").value)
        self.heading_noise_std = float(self.get_parameter("heading_noise_std").value)
        self.corridor_y_min = float(self.get_parameter("corridor_y_min").value)
        self.corridor_y_max = float(self.get_parameter("corridor_y_max").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        self.publish_corridor_walls = bool(self.get_parameter("publish_corridor_walls").value)

        try:
            import json
            self.vessels = json.loads(vessels_json)
        except Exception as e:
            self.get_logger().error(f"failed to parse vessels: {e}")
            self.vessels = []

        self.get_logger().info(f"dynamic obstacle node: {len(self.vessels)} vessels")

        # vessel state: {name: {x, y, yaw, vx, vy, target_x, target_y, arrived}}
        self.state = {}
        self.vessel_pubs = {}
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        if self.vessels:
            for v in self.vessels:
                name = v.get("name", "")
                if not name:
                    continue
                init_pose = v.get("init_pose", [0, 0, 0, 0])
                vel = v.get("velocity", [0, 0])
                target = v.get("target", [0, 0, 0])
                self.state[name] = {
                    "x": init_pose[0], "y": init_pose[1],
                    "yaw": init_pose[3] if len(init_pose) > 3 else 0.0,
                    "vx": vel[0], "vy": vel[1],
                    "base_speed": math.hypot(vel[0], vel[1]),
                    "tx": target[0], "ty": target[1],
                    "radius": v.get("radius", 1.1),
                     "mesh_resource": v.get("mesh_resource", ""),
                     "loop": bool(v.get("loop", True)),
                     "path": v.get("path", []),
                     "path_index": 0,
                }
        if not self.state:
            self.get_logger().warn("no vessel configs provided; dynamic obstacle node will stay idle")

        for name in self.state:
            self.vessel_pubs[name] = self.create_publisher(
                Odometry, f"/{name}/odom", 10)
            self.get_logger().info(
                f"  vessel {name}: ({self.state[name]['x']:.1f},{self.state[name]['y']:.1f}) "
                f"→ ({self.state[name]['tx']:.1f},{self.state[name]['ty']:.1f}) "
                f"vel=({self.state[name]['vx']:.2f},{self.state[name]['vy']:.2f}) "
                f"radius={self.state[name]['radius']:.1f}")

        timer_period = 1.0 / max(update_rate, 1.0)
        self.timer = self.create_timer(timer_period, self._tick)

    def _tick(self):
        now = self.get_clock().now()
        for name, s in self.state.items():
            dt = 0.1  # 10Hz timer

            if s["path"]:
                tgt = s["path"][s["path_index"]]
                s["tx"], s["ty"] = tgt[0], tgt[1]

            dx = s["tx"] - s["x"]
            dy = s["ty"] - s["y"]
            dist = math.hypot(dx, dy)
            if dist < 0.8:
                if s["path"]:
                    if s["path_index"] < len(s["path"]) - 1:
                        s["path_index"] += 1
                    elif s["loop"]:
                        s["path_index"] = 0
                elif s["loop"]:
                    s["tx"], s["ty"], s["x"], s["y"] = s["x"], s["y"], s["tx"], s["ty"]
                    dx = s["tx"] - s["x"]
                    dy = s["ty"] - s["y"]
                    dist = math.hypot(dx, dy)

            desired_yaw = math.atan2(dy, dx) if dist > 1e-6 else s["yaw"]
            noisy_speed = max(0.0, s["base_speed"] + random.gauss(0.0, self.speed_noise_std))
            noisy_yaw = desired_yaw + random.gauss(0.0, self.heading_noise_std)
            s["vx"] = noisy_speed * math.cos(noisy_yaw)
            s["vy"] = noisy_speed * math.sin(noisy_yaw)
            s["x"] += s["vx"] * dt
            s["y"] += s["vy"] * dt
            s["yaw"] = noisy_yaw

            # publish odometry
            odom = Odometry()
            odom.header = Header(stamp=now.to_msg(), frame_id=self.frame_id)
            odom.child_frame_id = f"{name}/base_link"
            odom.pose.pose.position = Point(x=s["x"], y=s["y"], z=0.0)
            odom.pose.pose.orientation = _quat_from_yaw(s["yaw"])
            odom.twist.twist = Twist()
            odom.twist.twist.linear.x = s["vx"]
            odom.twist.twist.linear.y = s["vy"]
            self.vessel_pubs[name].publish(odom)

        markers = MarkerArray()

        if self.publish_corridor_walls:
            wall_specs = [(self.corridor_y_min, 10000), (self.corridor_y_max, 10001)]
            for wy, wid in wall_specs:
                w = Marker()
                w.header = Header(stamp=now.to_msg(), frame_id=self.frame_id)
                w.ns = "corridor_walls"
                w.id = wid
                w.type = Marker.CUBE
                w.action = Marker.ADD
                w.pose.position = Point(x=37.5, y=wy, z=0.35)
                w.pose.orientation.w = 1.0
                w.scale.x = 105.0
                w.scale.y = 0.8
                w.scale.z = 0.7
                w.color.a = 1.0
                w.color.r = 0.08
                w.color.g = 0.08
                w.color.b = 0.08
                markers.markers.append(w)

        for idx, (name, s) in enumerate(self.state.items()):
            m = Marker()
            m.header = Header(stamp=now.to_msg(), frame_id=self.frame_id)
            m.ns = "dynamic_vessels"
            m.id = idx
            m.type = Marker.MESH_RESOURCE if s.get("mesh_resource") else Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position = Point(x=s["x"], y=s["y"], z=0.1)
            m.pose.orientation = _mesh_orientation(s.get("mesh_resource", ""), s["yaw"])
            d = 2.0 * float(s["radius"])
            m.scale.x = d
            m.scale.y = d
            m.scale.z = 1.0
            if s.get("mesh_resource"):
                m.mesh_resource = s["mesh_resource"]
                m.mesh_use_embedded_materials = True
                m.scale.x = 0.32
                m.scale.y = 0.32
                m.scale.z = 0.32
            else:
                m.color.a = 0.95
                m.color.r = 0.2
                m.color.g = 0.5
                m.color.b = 0.9
            markers.markers.append(m)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleVessel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
