#!/usr/bin/env python3
"""
Floating debris obstacles — a localized dense obstacle band for corridor avoidance.
Publishes Odometry on /debris_{i}/odom for each piece.

Design:
  - A moderate-size group appears in the mid-corridor interaction zone
  - Obstacles are small and visually closer to sub-meter discs
  - The group drifts laterally / diagonally as a cluster, creating one clear avoidance event
  - Respawn keeps them inside the interaction region instead of full-map random scattering
"""

import math
import random
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def _quat_from_yaw(yaw: float) -> Quaternion:
    h = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(h), w=math.cos(h))


class FloatingDebris(Node):
    def __init__(self):
        super().__init__("floating_debris_node")

        self.declare_parameter("num_debris", 12)
        self.declare_parameter("update_rate", 10.0)
        self.declare_parameter("seed", 0)
        self.declare_parameter("mesh_resource", "")
        self.declare_parameter("mesh_scale_x", 2.4)
        self.declare_parameter("mesh_scale_y", 1.5)
        self.declare_parameter("mesh_scale_z", 0.55)
        self.declare_parameter("mesh_z", 0.15)
        self.declare_parameter("marker_topic", "/debris_models")
        self.declare_parameter("frame_id", "odom")

        num = self.get_parameter("num_debris").value
        rate = self.get_parameter("update_rate").value
        seed_val = self.get_parameter("seed").value
        self.mesh_resource = str(self.get_parameter("mesh_resource").value)
        self.mesh_scale = [
            float(self.get_parameter("mesh_scale_x").value),
            float(self.get_parameter("mesh_scale_y").value),
            float(self.get_parameter("mesh_scale_z").value),
        ]
        self.mesh_z = float(self.get_parameter("mesh_z").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        rng = random.Random(seed_val)

        # Localized interaction zone placed directly across the main USV corridor.
        ZONE_X_MIN, ZONE_X_MAX = 1.0, 23.0
        ZONE_Y_MIN, ZONE_Y_MAX = -18.0, 18.0
        RESPAWN_X_MIN, RESPAWN_X_MAX = 1.5, 22.5
        RESPAWN_Y_MIN, RESPAWN_Y_MAX = -17.5, 17.5
        self.cluster_center_x = 11.5
        self.cluster_center_y = 1.5
        self.cluster_sigma_x = 7.2
        self.cluster_sigma_y = 11.0
        self.zone_x_min = ZONE_X_MIN
        self.zone_x_max = ZONE_X_MAX
        self.zone_y_min = ZONE_Y_MIN
        self.zone_y_max = ZONE_Y_MAX
        self.respawn_x_min = RESPAWN_X_MIN
        self.respawn_x_max = RESPAWN_X_MAX
        self.respawn_y_min = RESPAWN_Y_MIN
        self.respawn_y_max = RESPAWN_Y_MAX

        self.debris = []
        self.odom_publishers = {}
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 1)

        for i in range(num):
            name = f"debris_{i}"
            radius = rng.uniform(0.32, 0.46)
            x, y = self._sample_cluster_pose(rng)
            vx, vy = self._sample_cluster_velocity(rng, i)
            yaw = math.atan2(vy, vx)

            self.debris.append({
                "name": name, "radius": radius,
                "x": x, "y": y, "yaw": yaw,
                "vx": vx, "vy": vy,
            })
            self.odom_publishers[name] = self.create_publisher(
                Odometry, f"/{name}/odom", 10)

        self.rng = rng

        timer_period = 1.0 / max(rate, 1.0)
        self.timer = self.create_timer(timer_period, self._tick)

    def _sample_cluster_pose(self, rng):
        for _ in range(64):
            x = rng.gauss(self.cluster_center_x, self.cluster_sigma_x)
            y = rng.gauss(self.cluster_center_y, self.cluster_sigma_y)
            if self.zone_x_min <= x <= self.zone_x_max and self.zone_y_min <= y <= self.zone_y_max:
                return x, y
        return (
            rng.uniform(self.respawn_x_min, self.respawn_x_max),
            rng.uniform(self.respawn_y_min, self.respawn_y_max),
        )

    def _sample_cluster_velocity(self, rng, idx):
        lane = idx % 3
        if lane == 0:
            vx = rng.uniform(0.55, 0.82)
            vy = rng.uniform(0.14, 0.30)
        elif lane == 1:
            vx = rng.uniform(0.55, 0.82)
            vy = rng.uniform(-0.30, -0.14)
        else:
            vx = rng.uniform(0.68, 0.98)
            vy = rng.uniform(-0.16, 0.16)
            if abs(vy) < 0.06:
                vy = 0.08 if rng.random() < 0.5 else -0.08
        return vx, vy

    def _tick(self):
        now = self.get_clock().now()
        for d in self.debris:
            dt = 0.1

            # move
            d["x"] += d["vx"] * dt
            d["y"] += d["vy"] * dt

            if (d["x"] < self.zone_x_min - 2 or d["x"] > self.zone_x_max + 2 or
                d["y"] < self.zone_y_min - 1.5 or d["y"] > self.zone_y_max + 1.5):
                d["x"], d["y"] = self._sample_cluster_pose(self.rng)
                d["vx"], d["vy"] = self._sample_cluster_velocity(self.rng, int(d["name"].split("_")[-1]))
                d["yaw"] = math.atan2(d["vy"], d["vx"])

            # publish odometry
            odom = Odometry()
            odom.header = Header(stamp=now.to_msg(), frame_id=self.frame_id)
            odom.child_frame_id = f"{d['name']}/base_link"
            odom.pose.pose.position = Point(x=d["x"], y=d["y"], z=0.0)
            odom.pose.pose.orientation = _quat_from_yaw(d["yaw"])
            odom.twist.twist = Twist()
            odom.twist.twist.linear.x = d["vx"]
            odom.twist.twist.linear.y = d["vy"]
            self.odom_publishers[d["name"]].publish(odom)

        self._publish_markers(now)

    def _publish_markers(self, now):
        markers = MarkerArray()
        sx, sy, sz = self._marker_scale()

        for i, d in enumerate(self.debris):
            marker = Marker()
            marker.header = Header(stamp=now.to_msg(), frame_id=self.frame_id)
            marker.ns = "floating_debris"
            marker.id = i
            marker.action = Marker.ADD
            marker.pose.position = Point(x=d["x"], y=d["y"], z=self.mesh_z)
            marker.pose.orientation = _quat_from_yaw(d["yaw"])
            marker.scale.x = sx * d["radius"]
            marker.scale.y = sy * d["radius"]
            marker.scale.z = sz * d["radius"]
            marker.color.r = 0.72
            marker.color.g = 0.92
            marker.color.b = 1.0
            marker.color.a = 0.88
            marker.lifetime.sec = 1

            if self.mesh_resource:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = self.mesh_resource
                marker.mesh_use_embedded_materials = True
            else:
                marker.type = Marker.CUBE

            markers.markers.append(marker)

        self.marker_pub.publish(markers)

    def _marker_scale(self):
        vals = list(self.mesh_scale)
        while len(vals) < 3:
            vals.append(1.0)
        return float(vals[0]), float(vals[1]), float(vals[2])


def main(args=None):
    rclpy.init(args=args)
    node = FloatingDebris()
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
