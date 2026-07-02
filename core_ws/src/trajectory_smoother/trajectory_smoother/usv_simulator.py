import math

from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class UsvSimulator(Node):
    def __init__(self):
        super().__init__("usv_simulator")
        self.declare_parameter("init_pose", [-40.0, -40.0, 0.0])
        self.declare_parameter("speed", 1.9)
        self.declare_parameter("publish_rate", 20.0)

        init_pose = self.get_parameter("init_pose").value
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.x = float(init_pose[0])
        self.y = float(init_pose[1])
        self.z = float(init_pose[2]) if len(init_pose) > 2 else 0.0
        self.yaw = 0.0

        self.cmd_u = 0.0
        self.cmd_r = 0.0
        self._dbg_cmd_count = 0
        self._dbg_tick_count = 0

        ns = self.get_namespace().lstrip("/")
        self.base_frame = ns + "/base_link"
        self.agent_ns = ns if ns else "usv"

        self.sub_cmd_vel = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )
        self.pub_odom = self.create_publisher(Odometry, "odom", 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        period = 1.0 / publish_rate if publish_rate > 0 else 0.05
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(
            f"usv_simulator ready at ({self.x:.1f}, {self.y:.1f}), "
            f"cmd_vel driven, tf: odom -> {self.base_frame}"
        )

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_u = msg.linear.x
        self.cmd_r = msg.angular.z

    def tick(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if not hasattr(self, "_last_tick_time"):
            self._last_tick_time = now_sec
            return
        dt = min(now_sec - self._last_tick_time, 0.2)
        self._last_tick_time = now_sec

        self.yaw += self.cmd_r * dt
        self.yaw = self._wrap_angle(self.yaw)

        self.x += self.cmd_u * dt * math.cos(self.yaw)
        self.y += self.cmd_u * dt * math.sin(self.yaw)

        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation = self._quat_from_yaw()
        odom.twist.twist.linear.x = self.cmd_u
        odom.twist.twist.angular.z = self.cmd_r
        self.pub_odom.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = self.z
        tf_msg.transform.rotation = self._quat_from_yaw()
        self.tf_broadcaster.sendTransform(tf_msg)

    def _quat_from_yaw(self) -> Quaternion:
        return Quaternion(
            x=0.0, y=0.0,
            z=math.sin(self.yaw / 2.0),
            w=math.cos(self.yaw / 2.0),
        )

    @staticmethod
    def _wrap_angle(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = UsvSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
