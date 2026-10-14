#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

class N(Node):
    def __init__(self):
        super().__init__('dump_map')
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.m = None
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self,'m',m), q)

rclpy.init(); n = N()
while rclpy.ok() and n.m is None: rclpy.spin_once(n, timeout_sec=0.5)
m = n.m; r = m.info.resolution
ox, oy = m.info.origin.position.x, m.info.origin.position.y
W = m.info.width
print("raw /map, x:-1.5..1.5, y:+1.5 (top)..-1.5, 10 cm per char  (# occupied, ? unknown, . free)")
y = 1.5
while y >= -1.5:
    row = ""
    x = -1.5
    while x <= 1.5:
        c = m.data[int((y-oy)/r)*W + int((x-ox)/r)]
        row += "#" if c >= 65 else ("?" if c < 0 else ("+" if c > 0 else "."))
        x += 0.1
    print(f"{y:5.1f} {row}")
    y -= 0.1
rclpy.shutdown()
