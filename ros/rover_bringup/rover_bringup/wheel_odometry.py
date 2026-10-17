"""Wheel odometry from the ESP32's raw tick counts.

    /ticks_left, /ticks_right  (std_msgs/Int32, cumulative, signed)
        -> /wheel/odom          (nav_msgs/Odometry, odom -> base_footprint)
        -> /joint_states        (the four wheel joints, so RViz shows them turn)

This is the Pi side of INTEGRATION.md decision 2: odometry is computed here, not
on the ESP32, so the constants can be tuned without reflashing. It publishes no
TF by default - the EKF (config/ekf.yaml) owns odom -> base_footprint.

Two encoders, four wheels (INTEGRATION.md decision 1): both wheels on a side are
driven together and share one sensor, so the front and rear joints on a side get
the same angle.

Skid-steer note: the wheels slip sideways when turning, so the rover turns less
than the wheel difference suggests. effective_track_width corrects for it; get
it from the spin test in the package README. Heading comes mainly from the IMU
in the EKF anyway, which is why the yaw-rate covariance here is large.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

WHEEL_JOINTS = {
    'left': ('front_left_wheel_joint', 'rear_left_wheel_joint'),
    'right': ('front_right_wheel_joint', 'rear_right_wheel_joint'),
}


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        self.ticks_per_rev = self.declare_parameter('ticks_per_rev', 32.0).value  # 32-slot disc on the shaft extender
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.055).value  # 110 mm wheels
        # Centre of the left wheel to centre of the right wheel: 0.240 measured on the
        # bare motor shafts + 2 x 25 mm from the shaft extenders. Keep in step with
        # TRACK_WIDTH_M in the firmware and the URDF.
        self.track_width = self.declare_parameter('track_width', 0.290).value
        # Turning calibration for skid-steer. 0 = use track_width.
        eff = self.declare_parameter('effective_track_width', 0.0).value
        self.eff_track = eff if eff > 0.0 else self.track_width
        self.rate = self.declare_parameter('rate', 20.0).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.publish_tf = self.declare_parameter('publish_tf', False).value
        # The ESP32 publishes ticks at 20 Hz without pause. A gap longer than this
        # means the link dropped or the ESP32 rebooted (its counts restart at 0),
        # so the next counts are a fresh baseline, not motion.
        self.stale = self.declare_parameter('stale_timeout', 0.5).value
        # Belt and braces for a gap too short to notice: a jump faster than this
        # is not motion either.
        self.max_speed = self.declare_parameter('max_plausible_speed', 2.0).value

        self.m_per_tick = 2.0 * math.pi * self.wheel_radius / self.ticks_per_rev
        self.latest = {'left': None, 'right': None}
        self.used = {'left': None, 'right': None}
        self.x = self.y = self.yaw = 0.0
        self.last_t = None
        self.msg_t = None

        self.create_subscription(Int32, 'ticks_left', lambda m: self._on_ticks('left', m), 10)
        self.create_subscription(Int32, 'ticks_right', lambda m: self._on_ticks('right', m), 10)
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        if self.publish_tf:
            from tf2_ros import TransformBroadcaster
            self.tf = TransformBroadcaster(self)
        self.create_timer(1.0 / self.rate, self._update)
        self.get_logger().info(
            f'wheel radius {self.wheel_radius} m, track {self.track_width} m '
            f'(effective {self.eff_track} m), {self.ticks_per_rev} ticks/rev')

    def _on_ticks(self, side, msg):
        self.latest[side] = msg.data
        self.msg_t = self.get_clock().now()

    def _update(self):
        if None in self.latest.values():
            return  # wait until both sides have reported once
        now = self.get_clock().now()
        if (now - self.msg_t).nanoseconds * 1e-9 > self.stale:
            if self.last_t is not None:
                self.get_logger().warn('no ticks from the ESP32 - will re-sync when they return')
            self.last_t = None  # baseline again from the next counts
            return
        if self.last_t is None or None in self.used.values():
            self.used = dict(self.latest); self.last_t = now
            self._publish(now, 0.0, 0.0)
            return
        dt = (now - self.last_t).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        d_l = (self.latest['left'] - self.used['left']) * self.m_per_tick
        d_r = (self.latest['right'] - self.used['right']) * self.m_per_tick
        self.used = dict(self.latest); self.last_t = now

        if max(abs(d_l), abs(d_r)) > self.max_speed * max(dt, 1.0 / self.rate):
            self.get_logger().warn(
                f'tick jump of {d_l:.2f} / {d_r:.2f} m in {dt:.2f} s - ESP32 restarted? re-syncing')
            self._publish(now, 0.0, 0.0)
            return

        d_centre = (d_l + d_r) / 2.0
        d_yaw = (d_r - d_l) / self.eff_track
        # Integrate about the middle of the arc, as the firmware did.
        self.x += d_centre * math.cos(self.yaw + d_yaw / 2.0)
        self.y += d_centre * math.sin(self.yaw + d_yaw / 2.0)
        self.yaw = math.atan2(math.sin(self.yaw + d_yaw), math.cos(self.yaw + d_yaw))
        self._publish(now, d_centre / dt, d_yaw / dt)

    def _publish(self, now, v, w):
        stamp = now.to_msg()
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, self.odom_frame, self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        o.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
        # Pose drifts without bound, so the EKF fuses velocities only.
        # Skid-steer yaw rate is unreliable: large variance, the IMU gyro wins.
        for i, var in enumerate((0.01, 0.01, 1e6, 1e6, 1e6, 0.5)):
            o.pose.covariance[i * 7] = var
        for i, var in enumerate((0.002, 0.001, 1e6, 1e6, 1e6, 0.1)):
            o.twist.covariance[i * 7] = var
        self.odom_pub.publish(o)

        js = JointState()
        js.header.stamp = stamp
        for side, joints in WHEEL_JOINTS.items():
            angle = self.latest[side] / self.ticks_per_rev * 2.0 * math.pi
            for j in joints:
                js.name.append(j); js.position.append(angle)
        self.joint_pub.publish(js)

        if self.publish_tf:
            from geometry_msgs.msg import TransformStamped
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, self.odom_frame, self.base_frame
            t.transform.translation.x, t.transform.translation.y = self.x, self.y
            t.transform.rotation = o.pose.pose.orientation
            self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
