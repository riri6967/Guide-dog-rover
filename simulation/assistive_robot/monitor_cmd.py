#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class N(Node):
    def __init__(self):
        super().__init__('monitor_cmd')
        self.v = {'nav': (0.0, 0.0), 'smooth': (0.0, 0.0), 'out': (0.0, 0.0), 'odom': (0.0, 0.0)}
        for k, t in (('nav', '/cmd_vel_nav'), ('smooth', '/cmd_vel_smoothed'), ('out', '/cmd_vel')):
            self.create_subscription(Twist, t, lambda m, k=k: self.v.__setitem__(k, (m.linear.x, m.angular.z)), 10)
        self.create_subscription(Odometry, '/odom', lambda m: self.v.__setitem__('odom', (m.twist.twist.linear.x, m.twist.twist.angular.z)), 10)
        self.create_timer(0.5, self.p)
    def p(self):
        v = self.v
        print("nav v=%.2f w=%.2f | smooth v=%.2f w=%.2f | OUT v=%.2f w=%.2f | actual v=%.2f w=%.2f" %
              (*v['nav'], *v['smooth'], *v['out'], *v['odom']), flush=True)

rclpy.init(); rclpy.spin(N())
