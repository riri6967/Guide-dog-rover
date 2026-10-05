#!/usr/bin/env python3
"""
Live one-line view of what the rover is being told and what it reports back.

Run alongside the agent to confirm motors and encoders are cooperating:

    python3 rover_monitor.py

Output, twice a second:

    cmd +0.50 m/s +0.00 rad/s | ticks L   1247 (+38)  R   1193 (+41) | MOVING | ...

The bracketed numbers are ticks since the previous line. That rate of change is
the useful signal — a frozen total alongside a live command means the wheels
are not turning, which is the failure worth spotting quickly.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32


class Monitor(Node):
    def __init__(self):
        super().__init__('rover_monitor')
        self.lin = self.ang = 0.0
        self.tl = self.tr = 0
        self.prev_tl = self.prev_tr = 0
        self.x = self.y = self.th = 0.0

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Int32, 'ticks_left', self.on_tl, 10)
        self.create_subscription(Int32, 'ticks_right', self.on_tr, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(0.5, self.report)

    def on_cmd(self, m):
        self.lin, self.ang = m.linear.x, m.angular.z

    def on_tl(self, m):
        self.tl = m.data

    def on_tr(self, m):
        self.tr = m.data

    def on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.th = math.degrees(2.0 * math.atan2(q.z, q.w))

    def report(self):
        dl = self.tl - self.prev_tl
        dr = self.tr - self.prev_tr
        self.prev_tl, self.prev_tr = self.tl, self.tr

        moving = "MOVING" if (dl or dr) else "  --  "
        print(f"cmd {self.lin:+5.2f} m/s {self.ang:+5.2f} rad/s | "
              f"ticks L {self.tl:7d} ({dl:+4d})  R {self.tr:7d} ({dr:+4d}) | "
              f"{moving} | odom {self.x:+6.2f} {self.y:+6.2f} {self.th:+6.1f}deg",
              flush=True)


def main():
    rclpy.init()
    try:
        rclpy.spin(Monitor())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
