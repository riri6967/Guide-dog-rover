import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Relay(Node):
    def __init__(self):
        super().__init__("temp_sac_relay")

        self.pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.sub = self.create_subscription(
            Twist,
            "/sac_cmd_vel",
            self.cb,
            10,
        )

    def cb(self, msg):
        self.pub.publish(msg)


rclpy.init()
node = Relay()

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
