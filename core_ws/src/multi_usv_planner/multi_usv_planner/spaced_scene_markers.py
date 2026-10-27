#!/usr/bin/env python3
"""Publish RViz markers for the spaced (old-scene-stretched) scenario."""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import json


class SpacedSceneMarkers(Node):
    def __init__(self):
        super().__init__('spaced_scene_markers')
        usv_cfg_param = self.declare_parameter('usv_configs', '[]')
        static_x_param = self.declare_parameter('static_obstacles_x', [0.0])
        static_y_param = self.declare_parameter('static_obstacles_y', [0.0])
        static_r_param = self.declare_parameter('static_obstacles_r', [0.0])
        publish_rate_param = self.declare_parameter('publish_rate', 2.0)
        marker_topic_param = self.declare_parameter('marker_topic', '/spaced_scene/scene_markers')
        static_mesh_param = self.declare_parameter('static_mesh_resource', 'file:///home/lu/paper2/core_ws/src/model/models/meshes/ice_floes.glb')
        use_mesh_param = self.declare_parameter('use_static_mesh', False)
        static_shape_param = self.declare_parameter('static_shape', 'island')

        self.usv_configs = json.loads(usv_cfg_param.value)
        self.static_x = list(static_x_param.value)
        self.static_y = list(static_y_param.value)
        self.static_r = list(static_r_param.value)
        self.static_mesh_resource = str(static_mesh_param.value)
        self.use_mesh = bool(use_mesh_param.value)
        self.static_shape = str(static_shape_param.value)

        self.pub = self.create_publisher(MarkerArray, str(marker_topic_param.value), 1)
        rate = float(publish_rate_param.value)
        self.timer = self.create_timer(1.0 / rate, self.publish)
        self.publish()  # immediate

    def publish(self):
        arr = MarkerArray()
        mid = 0

        # The repository's GLBs are complete ice/water scenes rather than
        # isolated reef meshes.  The default therefore uses a flattened,
        # earthy mound marker as a stable island proxy; an explicit mesh can
        # still be enabled for inspection with use_static_mesh:=true.
        for i in range(len(self.static_x)):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'static_obstacles_mesh'
            m.id = mid
            mid += 1
            m.type = Marker.MESH_RESOURCE if self.use_mesh else (
                Marker.SPHERE if self.static_shape == 'island' else Marker.CYLINDER)
            m.action = Marker.ADD
            m.pose.position.x = self.static_x[i]
            m.pose.position.y = self.static_y[i]
            m.pose.position.z = 0.20
            m.pose.orientation.w = 1.0
            r = self.static_r[i] if i < len(self.static_r) else 1.0
            mesh_scale = max(1.0, r * 2.2) if not self.use_mesh else max(0.45, r * 0.75)
            m.scale.x = mesh_scale
            m.scale.y = mesh_scale
            m.scale.z = mesh_scale if self.use_mesh else 0.55
            m.color.r = 0.34
            m.color.g = 0.27
            m.color.b = 0.14
            m.color.a = 0.95
            if self.use_mesh:
                m.mesh_resource = self.static_mesh_resource
                m.mesh_use_embedded_materials = True
            m.lifetime.sec = 3
            arr.markers.append(m)

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SpacedSceneMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
