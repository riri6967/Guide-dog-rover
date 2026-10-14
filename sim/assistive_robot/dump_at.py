#!/usr/bin/env python3
# usage: dump_at.py cx cy half_width_m   (global costmap; # blocked, + inflated, . free)
import sys, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

cx, cy, hw = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
rclpy.init(); n = Node('dump_at'); box = []
q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: box.append(m), q)
while rclpy.ok() and not box: rclpy.spin_once(n, timeout_sec=0.5)
m = box[0]; r = m.info.resolution
ox, oy, W, H = m.info.origin.position.x, m.info.origin.position.y, m.info.width, m.info.height
print(f"global costmap around ({cx},{cy}), x {cx-hw:.1f}..{cx+hw:.1f} left to right, y {cy+hw:.1f} (top) .. {cy-hw:.1f}, 10 cm per char")
y = cy + hw
while y >= cy - hw:
    row = ""; x = cx - hw
    while x <= cx + hw:
        i, j = int((x-ox)/r), int((y-oy)/r)
        c = m.data[j*W+i] if 0 <= i < W and 0 <= j < H else 100
        row += "#" if c >= 90 else ("+" if c > 0 else ".")
        x += 0.1
    print(f"{y:5.1f} {row}"); y -= 0.1
rclpy.shutdown()
