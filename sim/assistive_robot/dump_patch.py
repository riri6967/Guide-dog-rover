#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry

class N(Node):
    def __init__(self):
        super().__init__('dump_patch')
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cm = None; self.od = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: setattr(self,'cm',m), q)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self,'od',m), 10)

rclpy.init(); n = N()
while rclpy.ok() and (n.cm is None or n.od is None): rclpy.spin_once(n, timeout_sec=0.5)
p = n.od.pose.pose.position
print(f"odom position: x={p.x:.2f} y={p.y:.2f}")
m = n.cm; r = m.info.resolution
ox, oy = m.info.origin.position.x, m.info.origin.position.y
W = m.info.width
print("patch x:-1.5..1.5 (left to right), y: +1.5 (top) .. -1.5 (bottom), 10 cm per char")
y = 1.5
while y >= -1.5:
    row = ""
    x = -1.5
    while x <= 1.5:
        c = m.data[int((y-oy)/r)*W + int((x-ox)/r)]
        row += "#" if c >= 90 else ("+" if c > 0 else ".")
        x += 0.1
    print(f"{y:5.1f} {row}")
    y -= 0.1
rclpy.shutdown()
