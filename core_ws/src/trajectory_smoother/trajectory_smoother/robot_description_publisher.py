from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.node import Node
import rclpy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__("robot_description_publisher")
        self.declare_parameter("robot_description", "")

        description = str(self.get_parameter("robot_description").value)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.publisher = self.create_publisher(String, "robot_description", qos)
        self.timer = self.create_timer(0.5, self.publish_once)
        self.description = description
        self.published = False

    def publish_once(self):
        if self.published or not self.description:
            return
        msg = String()
        msg.data = self.description
        self.publisher.publish(msg)
        self.published = True
        self.get_logger().info("Published robot_description topic for RViz RobotModel.")


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
