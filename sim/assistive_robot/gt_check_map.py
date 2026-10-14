#!/usr/bin/env python3
import math, subprocess, re, sys, time, rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

rclpy.init(); n = Node('gt_check_map')
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
t0 = time.time()
while time.time() - t0 < 20 and not buf.can_transform('map', 'base_link', Time()):
    rclpy.spin_once(n, timeout_sec=0.2)
t1 = time.time()
while time.time() - t1 < 4:                      # keep listening so AMCL's map->odom arrives
    rclpy.spin_once(n, timeout_sec=0.1)
o = buf.lookup_transform('map', 'odom', Time()).transform.translation
if abs(o.x) < 1e-6 and abs(o.y) < 1e-6:
    print("WARNING: map->odom is exactly identity, so this is raw odometry, not AMCL")
tf = buf.lookup_transform('map', 'base_link', Time()).transform
q = tf.rotation; nyaw = math.degrees(2*math.atan2(q.z, q.w))
out = subprocess.run("gz topic -e -n 1 -t /world/assistive_world/pose/info | grep -A14 'name: \"assistive_rover\"'",
                     shell=True, capture_output=True, text=True).stdout
pos = out.split('position')[1].split('orientation')[0]; ori = out.split('orientation')[1]
gx = float(re.search(r'x: (\S+)', pos).group(1)); gy = float(re.search(r'y: (\S+)', pos).group(1))
gyaw = math.degrees(2*math.atan2(float(re.search(r'z: (\S+)', ori).group(1)), float(re.search(r'w: (\S+)', ori).group(1))))
dyaw = (nyaw - gyaw + 180) % 360 - 180
print(f"nav2=({tf.translation.x:.2f},{tf.translation.y:.2f}) yaw={nyaw:.0f}  real=({gx:.2f},{gy:.2f}) yaw={gyaw:.0f}  "
      f"pos error={math.hypot(tf.translation.x-gx, tf.translation.y-gy):.2f} m  yaw error={dyaw:.0f} deg")
rclpy.shutdown()
