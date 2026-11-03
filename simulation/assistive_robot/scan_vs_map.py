#!/usr/bin/env python3
import math, re, subprocess, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

class N(Node):
    def __init__(self):
        super().__init__('scan_vs_map')
        self.m = None; self.s = None
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'm', m), q)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 's', m), qos_profile_sensor_data)

rclpy.init(); n = N()
while rclpy.ok() and (n.m is None or n.s is None): rclpy.spin_once(n, timeout_sec=0.5)
out = subprocess.run("gz topic -e -n 1 -t /world/assistive_world/pose/info | grep -A14 'name: \"assistive_rover\"'",
                     shell=True, capture_output=True, text=True).stdout
pos = out.split('position')[1].split('orientation')[0]; ori = out.split('orientation')[1]
gx = float(re.search(r'x: (\S+)', pos).group(1)); gy = float(re.search(r'y: (\S+)', pos).group(1))
gyaw = 2*math.atan2(float(re.search(r'z: (\S+)', ori).group(1)), float(re.search(r'w: (\S+)', ori).group(1)))
print(f"real pose: ({gx:.2f}, {gy:.2f}) yaw={math.degrees(gyaw):.0f} deg")

m, s = n.m, n.s; r = m.info.resolution
ox, oy, W, H = m.info.origin.position.x, m.info.origin.position.y, m.info.width, m.info.height
def near_wall(x, y, k=2):
    i, j = int((x-ox)/r), int((y-oy)/r)
    for dj in range(-k, k+1):
        for di in range(-k, k+1):
            a, b = i+di, j+dj
            if 0 <= a < W and 0 <= b < H and m.data[b*W+a] >= 65: return True
    return False

hits = [(s.angle_min + i*s.angle_increment, rg) for i, rg in enumerate(s.ranges)
        if math.isfinite(rg) and s.range_min < rg < s.range_max]
print(f"valid hits: {len(hits)}")
for off in (0, 90, 180, 270):
    ok = sum(near_wall(gx + rg*math.cos(a + gyaw + math.radians(off)),
                       gy + rg*math.sin(a + gyaw + math.radians(off))) for a, rg in hits)
    print(f"scan rotated {off:3d} deg: {100*ok/len(hits):5.1f}% of hits on map walls")
rclpy.shutdown()
