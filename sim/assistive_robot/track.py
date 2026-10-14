#!/usr/bin/env python3
import math, re, subprocess, sys, time, rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60
rclpy.init(); n = Node('track')
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
yaw = lambda q: math.degrees(2*math.atan2(q.z, q.w))
t0 = time.time()
print("  t(s) | nav2 x, y, yaw        | real x, y, yaw        | pos err | map->odom x, y, yaw")
while time.time() - t0 < dur:
    for _ in range(5): rclpy.spin_once(n, timeout_sec=0.05)
    try:
        b = buf.lookup_transform('map', 'base_link', Time()).transform
        o = buf.lookup_transform('map', 'odom', Time()).transform
        out = subprocess.run("gz topic -e -n 1 -t /world/assistive_world/pose/info | grep -A14 'name: \"assistive_rover\"'",
                             shell=True, capture_output=True, text=True).stdout
        pos = out.split('position')[1].split('orientation')[0]; ori = out.split('orientation')[1]
        gx = float(re.search(r'x: (\S+)', pos).group(1)); gy = float(re.search(r'y: (\S+)', pos).group(1))
        gyaw = math.degrees(2*math.atan2(float(re.search(r'z: (\S+)', ori).group(1)), float(re.search(r'w: (\S+)', ori).group(1))))
        err = math.hypot(b.translation.x-gx, b.translation.y-gy)
        print(f"{time.time()-t0:6.1f} | ({b.translation.x:6.2f},{b.translation.y:6.2f}) {yaw(b.rotation):5.0f} | "
              f"({gx:6.2f},{gy:6.2f}) {gyaw:5.0f} | {err:6.2f}  | ({o.translation.x:6.2f},{o.translation.y:6.2f}) {yaw(o.rotation):5.0f}", flush=True)
    except Exception as e:
        print("waiting:", str(e)[:60], flush=True)
rclpy.shutdown()
