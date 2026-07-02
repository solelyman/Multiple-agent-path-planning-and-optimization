from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__("odom_tf_broadcaster")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("parent_frame", "odom")

        ns = self.get_namespace().lstrip("/")
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = ns + "/base_link"
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.latest_pose = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub_odom = self.create_subscription(
            Odometry, "odom", self.odom_callback, 20
        )
        period = 1.0 / publish_rate if publish_rate > 0 else 0.1
        self.timer = self.create_timer(period, self.publish_tf)
        self.get_logger().info(
            f"odom_tf_broadcaster ready: {self.parent_frame} -> {self.child_frame}"
        )

    def odom_callback(self, msg: Odometry):
        self.latest_pose = msg.pose.pose

    def publish_tf(self):
        if self.latest_pose is None:
            return
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        p = self.latest_pose
        transform.transform.translation.x = p.position.x
        transform.transform.translation.y = p.position.y
        transform.transform.translation.z = p.position.z
        transform.transform.rotation = p.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
