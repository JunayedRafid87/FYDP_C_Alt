#!/usr/bin/env python3
"""
Odometry Gate Node — FYDP Cv2
=============================
Owns the ``odom`` ➔ ``base_link`` transform and gates it on the IMU-derived scan state.

RF2O estimates planar motion from consecutive laser scans, but it only sees the scans
the mux lets through, and pure scan matching keeps jittering by a few millimetres even
when the table is perfectly still. Both of those are unacceptable for the 3D sweep:
the accumulated point cloud smears if the frame it is projected into wanders.

So RF2O runs with ``publish_tf:=false`` and this node is the single owner of the edge:

* ``MOVING`` / ``STABILIZING`` — track RF2O. The axis follows the table around.
* ``SCANNING`` / ``SCAN_COMPLETE`` — frozen. The axis is nailed in place, and stays
  nailed until the IMU reports motion again and the ESP32 drops back to ``MOVING``.

On the ``STABILIZING`` ➔ ``SCANNING`` edge the pose is latched to the *average* of the
last ``settle_window_sec`` of tracking, so the seven seconds of flat 2D scanning are
what decides the position, rather than whichever single scan happened to land last.

Because RF2O is blind (and its internal dt therefore meaningless) across the ~37 s the
table is stationary, this node never consumes RF2O's absolute pose. It composes RF2O's
*relative* motion since the last re-baseline onto its own anchor, and re-baselines
whenever tracking resumes. RF2O is free to jump; the published TF stays continuous.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
    """Extract the yaw angle from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class OdomGateNode(Node):
    def __init__(self):
        super().__init__('odom_gate_node')

        self.declare_parameter('odom_in_topic', '/odom_rf2o')
        self.declare_parameter('odom_out_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 50.0)
        # How much of the flat 2D settling phase to average when latching the pose.
        self.declare_parameter('settle_window_sec', 3.0)
        # A single RF2O step larger than this is a scan-matching glitch, not real motion.
        self.declare_parameter('max_step_m', 0.50)
        self.declare_parameter('max_step_rad', 1.05)  # ~60 degrees
        self.declare_parameter('freeze_states', ['SCANNING', 'SCAN_COMPLETE'])
        self.declare_parameter('publish_status_marker', True)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.settle_window = self.get_parameter('settle_window_sec').value
        self.max_step_m = self.get_parameter('max_step_m').value
        self.max_step_rad = self.get_parameter('max_step_rad').value
        self.freeze_states = [s.upper() for s in self.get_parameter('freeze_states').value]
        self.want_marker = self.get_parameter('publish_status_marker').value
        rate = self.get_parameter('publish_rate').value

        # Published pose (the pose the TF actually carries)
        self.pub_x = 0.0
        self.pub_y = 0.0
        self.pub_yaw = 0.0

        # Re-baseline bookkeeping: published pose and RF2O pose at the last baseline
        self.anchor_x = 0.0
        self.anchor_y = 0.0
        self.anchor_yaw = 0.0
        self.ref_x = 0.0
        self.ref_y = 0.0
        self.ref_yaw = 0.0
        self.have_ref = False

        # Last raw RF2O sample, for glitch rejection
        self.prev_raw = None

        self.frozen = False
        self.state = 'MOVING'
        self.twist = (0.0, 0.0)

        # Rolling history of tracked poses, used to average out the latch
        self.history = deque()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_out_topic').value, 10)
        self.marker_pub = self.create_publisher(Marker, '/scan_status_marker', 1)

        self.create_subscription(
            Odometry, self.get_parameter('odom_in_topic').value, self.odom_callback, 10)
        self.create_subscription(String, '/scan_state', self.state_callback, 10)

        self.create_timer(1.0 / rate, self.publish_tf)

        self.get_logger().info(
            f'Odom gate started — owning {self.odom_frame} -> {self.base_frame} at {rate:.0f} Hz')
        self.get_logger().info(f'Freezing in states: {self.freeze_states}')

    # ------------------------------------------------------------------ state

    def state_callback(self, msg):
        state = msg.data.strip().upper()
        if state == self.state:
            return
        self.state = state

        should_freeze = state in self.freeze_states
        if should_freeze and not self.frozen:
            self._latch()
        elif not should_freeze and self.frozen:
            self._release()

    def _latch(self):
        """Freeze the TF, snapping to the average of the recent settled poses."""
        avg = self._average_history()
        if avg is not None:
            self.pub_x, self.pub_y, self.pub_yaw = avg
        self.frozen = True
        self.twist = (0.0, 0.0)
        self.history.clear()
        self.get_logger().info(
            f'{self.state}: TF FROZEN at x={self.pub_x:.3f} y={self.pub_y:.3f} '
            f'yaw={math.degrees(self.pub_yaw):.1f}deg — will not move until the IMU reports motion')

    def _release(self):
        """Resume tracking RF2O, re-baselined onto wherever the frozen pose sits."""
        self.frozen = False
        self.have_ref = False   # forces a re-baseline on the next RF2O sample
        self.prev_raw = None
        self.history.clear()
        self.get_logger().info(
            f'{self.state}: motion detected — TF tracking resumed from '
            f'x={self.pub_x:.3f} y={self.pub_y:.3f}')

    def _average_history(self):
        """Mean of the poses inside the settle window (circular mean for yaw)."""
        if not self.history:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        recent = [h for h in self.history if now - h[0] <= self.settle_window]
        if not recent:
            recent = [self.history[-1]]
        n = float(len(recent))
        mx = sum(h[1] for h in recent) / n
        my = sum(h[2] for h in recent) / n
        syaw = sum(math.sin(h[3]) for h in recent) / n
        cyaw = sum(math.cos(h[3]) for h in recent) / n
        self.get_logger().info(f'Latching pose averaged over {len(recent)} settled 2D scans')
        return mx, my, math.atan2(syaw, cyaw)

    # ------------------------------------------------------------------ odom

    def odom_callback(self, msg):
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        ryaw = yaw_from_quat(msg.pose.pose.orientation)

        if self.frozen:
            # Stay quiet, but remember where RF2O is so the next baseline is fresh.
            self.prev_raw = (rx, ry, ryaw)
            return

        # Reject scan-matching glitches (typical right after a long blind window).
        if self.prev_raw is not None:
            step = math.hypot(rx - self.prev_raw[0], ry - self.prev_raw[1])
            dyaw = abs(wrap(ryaw - self.prev_raw[2]))
            if step > self.max_step_m or dyaw > self.max_step_rad:
                self.get_logger().warn(
                    f'Rejecting RF2O jump ({step:.2f} m, {math.degrees(dyaw):.1f} deg) — re-baselining')
                self.have_ref = False
        self.prev_raw = (rx, ry, ryaw)

        if not self.have_ref:
            self.ref_x, self.ref_y, self.ref_yaw = rx, ry, ryaw
            self.anchor_x, self.anchor_y, self.anchor_yaw = self.pub_x, self.pub_y, self.pub_yaw
            self.have_ref = True
            return

        # Compose RF2O's motion since the baseline onto the anchor pose.
        dx = rx - self.ref_x
        dy = ry - self.ref_y
        rot = self.anchor_yaw - self.ref_yaw
        c, s = math.cos(rot), math.sin(rot)

        self.pub_x = self.anchor_x + c * dx - s * dy
        self.pub_y = self.anchor_y + s * dx + c * dy
        self.pub_yaw = wrap(self.anchor_yaw + wrap(ryaw - self.ref_yaw))
        self.twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

        stamp = self.get_clock().now().nanoseconds * 1e-9
        self.history.append((stamp, self.pub_x, self.pub_y, self.pub_yaw))
        while self.history and stamp - self.history[0][0] > self.settle_window * 2.0:
            self.history.popleft()

    # ------------------------------------------------------------------ output

    def publish_tf(self):
        """Emit the gated transform and odometry at a fixed rate, frozen or not."""
        now = self.get_clock().now().to_msg()
        qz = math.sin(self.pub_yaw / 2.0)
        qw = math.cos(self.pub_yaw / 2.0)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.pub_x
        t.transform.translation.y = self.pub_y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.pub_x
        odom.pose.pose.position.y = self.pub_y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = 0.0 if self.frozen else self.twist[0]
        odom.twist.twist.angular.z = 0.0 if self.frozen else self.twist[1]
        self.odom_pub.publish(odom)

        if self.want_marker:
            self._publish_marker(now)

    def _publish_marker(self, now):
        m = Marker()
        m.header.stamp = now
        m.header.frame_id = self.base_frame
        m.ns = 'scan_status'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.z = 0.35
        m.pose.orientation.w = 1.0
        m.scale.z = 0.12
        m.color.a = 1.0
        if self.state == 'SCANNING':
            m.color.r, m.color.g, m.color.b = 0.1, 1.0, 0.2      # green: 3D sweep
        elif self.state == 'STABILIZING':
            m.color.r, m.color.g, m.color.b = 1.0, 0.85, 0.1     # amber: settling
        elif self.state == 'MOVING':
            m.color.r, m.color.g, m.color.b = 1.0, 0.3, 0.3      # red: tracking
        else:
            m.color.r, m.color.g, m.color.b = 0.6, 0.6, 1.0      # blue: idle
        m.text = f'{self.state}  |  TF {"FROZEN" if self.frozen else "TRACKING"}'
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = OdomGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
