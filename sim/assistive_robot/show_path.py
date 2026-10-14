#!/usr/bin/env python3
import math, sys, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose

gx, gy = float(sys.argv[1]), float(sys.argv[2])
sx, sy = (float(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 4 else (0.0, 0.0)
rclpy.init(); n = Node('show_path')
c = ActionClient(n, ComputePathToPose, 'compute_path_to_pose'); c.wait_for_server()
g = ComputePathToPose.Goal(); g.planner_id = 'GridBased'; g.use_start = True
for p, (x, y) in ((g.goal, (gx, gy)), (g.start, (sx, sy))):
    p.header.frame_id = 'map'; p.pose.position.x = x; p.pose.position.y = y; p.pose.orientation.w = 1.0
f = c.send_goal_async(g); rclpy.spin_until_future_complete(n, f)
r = f.result().get_result_async(); rclpy.spin_until_future_complete(n, r)
pts = [(p.pose.position.x, p.pose.position.y) for p in r.result().result.path.poses]
L = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(pts, pts[1:]))
print(f"goal ({gx},{gy}) from ({sx},{sy}): {len(pts)} poses, path length {L:.1f} m, straight line {math.hypot(gx-sx, gy-sy):.1f} m")
step = max(1, len(pts)//14)
print("waypoints:", " -> ".join(f"({x:.1f},{y:.1f})" for x, y in pts[::step]))
rclpy.shutdown()
