#!/usr/bin/env python3
"""
Lightweight dynamic obstacle vessel simulator.
Each vessel moves from its initial position toward its target in a straight line,
publishing Odometry for the planner to consume.
No Gazebo dependency.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Header


def _quat_from_yaw(yaw: float) -> Quaternion:
    h = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(h), w=math.cos(h))


class DynamicObstacleVessel(Node):
    def __init__(self):
        super().__init__("dynamic_obstacle_node")

        self.declare_parameter("vessels", "[]")
        self.declare_parameter("update_rate", 10.0)
        self.declare_parameter("frame_id", "odom")

        vessels_json = self.get_parameter("vessels").value
        update_rate = self.get_parameter("update_rate").value
        self.frame_id = str(self.get_parameter("frame_id").value)

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

        # Radius convention: half the longest length of the ship model (for RViz visualization).
        # These defaults are used only if not overridden by launch.py JSON config.
        default_configs = {
            "vessel_e1": {"x": 4.0, "y": -8.2, "yaw": 1.57, "vx": 0.0, "vy": 0.92,
                          "tx": 4.0, "ty": 9.5, "radius": 3.0},
            "vessel_e2": {"x": 16.0, "y": -7.6, "yaw": 1.57, "vx": 0.0, "vy": 0.94,
                          "tx": 16.0, "ty": 10.5, "radius": 3.0},
            "vessel_f1": {"x": 10.0, "y": 11.2, "yaw": -1.57, "vx": 0.0, "vy": -0.88,
                          "tx": 10.0, "ty": -6.0, "radius": 3.5},
            "vessel_f2": {"x": 22.0, "y": 12.0, "yaw": -1.57, "vx": 0.0, "vy": -0.86,
                          "tx": 22.0, "ty": -5.5, "radius": 3.5},
        }

        used_names = set()
        if self.vessels:
            for v in self.vessels:
                name = v.get("name", "")
                if not name:
                    continue
                used_names.add(name)
                init_pose = v.get("init_pose", [0, 0, 0, 0])
                vel = v.get("velocity", [0, 0])
                target = v.get("target", [0, 0, 0])
                self.state[name] = {
                    "x": init_pose[0], "y": init_pose[1],
                    "yaw": init_pose[3] if len(init_pose) > 3 else 0.0,
                    "vx": vel[0], "vy": vel[1],
                    "tx": target[0], "ty": target[1],
                    "radius": v.get("radius", 3.0),
                    "arrived": False,
                }
        # Add defaults not explicitly configured
        for name, cfg in default_configs.items():
            if name not in used_names:
                self.state[name] = {
                    "x": cfg["x"], "y": cfg["y"], "yaw": cfg["yaw"],
                    "vx": cfg["vx"], "vy": cfg["vy"],
                    "tx": cfg["tx"], "ty": cfg["ty"],
                    "radius": cfg["radius"], "arrived": False,
                }

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
            if s["arrived"]:
                continue

            # move toward target
            dx = s["tx"] - s["x"]
            dy = s["ty"] - s["y"]
            dist = math.hypot(dx, dy)
            if dist < 0.5:
                s["arrived"] = True
                self.get_logger().info(f"  vessel {name} arrived at target")
                s["x"] = s["tx"]
                s["y"] = s["ty"]
            else:
                dt = 0.1  # 10Hz timer
                s["x"] += s["vx"] * dt
                s["y"] += s["vy"] * dt
                s["yaw"] = math.atan2(s["vy"], s["vx"]) if abs(s["vx"]) > 1e-6 or abs(s["vy"]) > 1e-6 else s["yaw"]

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
