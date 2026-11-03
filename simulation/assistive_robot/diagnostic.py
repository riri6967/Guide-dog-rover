#!/usr/bin/env python3
# usage: /usr/bin/python3 check_line.py x0 y0 x1 y1
import sys, math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

x0, y0, x1, y1 = map(float, sys.argv[1:5])
BLOCK = 90  # costmap topic: inscribed ~99, lethal 100

class N(Node):
    def __init__(self):
        super().__init__('check_line')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.msg = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.cb, qos)
    def cb(self, m): self.msg = m

rclpy.init(); n = N()
while rclpy.ok() and n.msg is None: rclpy.spin_once(n, timeout_sec=0.5)
m = n.msg; r = m.info.resolution
ox, oy = m.info.origin.position.x, m.info.origin.position.y
W, H = m.info.width, m.info.height
def cost(x, y):
    i, j = int((x-ox)/r), int((y-oy)/r)
    return m.data[j*W+i] if 0 <= i < W and 0 <= j < H else None

print(f"start cost: {cost(x0,y0)}   goal cost: {cost(x1,y1)}")
# worst cost within 0.35 m of start
worst = max((cost(x0+dx*r, y0+dy*r) or 0, (round(x0+dx*r,2), round(y0+dy*r,2)))
            for dx in range(-7, 8) for dy in range(-7, 8))
print("worst cost within 0.35 m of start:", worst)

L = math.hypot(x1-x0, y1-y0); steps = int(L/r)
blocked = []
for k in range(steps+1):
    x = x0 + (x1-x0)*k/steps; y = y0 + (y1-y0)*k/steps
    c = cost(x, y)
    if c is not None and c >= BLOCK: blocked.append((round(x,2), round(y,2), c))
print(f"blocked cells on straight line: {len(blocked)}")
for b in blocked[:15]: print("  ", b)
rclpy.shutdown()
