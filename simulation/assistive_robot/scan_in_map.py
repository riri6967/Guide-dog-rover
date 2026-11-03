#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros

class N(Node):
    def __init__(self):
        super().__init__('scan_in_map')
        self.buf = tf2_ros.Buffer(); self.l = tf2_ros.TransformListener(self.buf, self)
        self.s = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self,'s',m), qos_profile_sensor_data)

rclpy.init(); n = N()
while rclpy.ok() and n.s is None: rclpy.spin_once(n, timeout_sec=0.5)
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.2)
s = n.s
print("scan frame_id:", s.header.frame_id)
t = n.buf.lookup_transform('map', s.header.frame_id, Time())
tr, q = t.transform.translation, t.transform.rotation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
print(f"lidar in map: x={tr.x:.2f} y={tr.y:.2f} z={tr.z:.2f} yaw={math.degrees(yaw):.1f} deg")
print(f"scan: angle_min={math.degrees(s.angle_min):.1f} angle_max={math.degrees(s.angle_max):.1f} n={len(s.ranges)}")
pts = []
for i, rg in enumerate(s.ranges):
    if math.isfinite(rg) and s.range_min < rg < s.range_max:
        a = s.angle_min + i*s.angle_increment + yaw
        pts.append((tr.x + rg*math.cos(a), tr.y + rg*math.sin(a)))
box = [p for p in pts if abs(p[0]) < 1.5 and abs(p[1]) < 1.2]
print(f"valid hits: {len(pts)}   hits inside origin box (|x|<1.5,|y|<1.2): {len(box)}")
for p in box[:10]: print(f"   ({p[0]:.2f}, {p[1]:.2f})")
rclpy.shutdown()
