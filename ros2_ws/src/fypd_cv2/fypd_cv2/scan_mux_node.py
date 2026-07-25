#!/usr/bin/env python3
"""
Scan Mux Node — FYDP Cv2
=========================
Routes the raw '/scan' topic to '/scan_slam' only when the stepper motor is
flat (during MOVING and STABILIZING states). This blinds Google Cartographer
and RF2O during the 3D pitching sweep to prevent map corruption.

SCAN_COMPLETE is gated too. The scans are flat again by then, but the whole TF
chain is deliberately frozen until the IMU reports motion — letting Cartographer
keep matching would let it nudge map -> odom underneath a frame that is supposed
to be standing perfectly still.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# States in which scans are withheld from the 2D SLAM / odometry pipeline.
GATED_STATES = ("SCANNING", "SCAN_COMPLETE")


class ScanMuxNode(Node):
    def __init__(self):
        super().__init__('scan_mux_node')

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )
        self.state_sub = self.create_subscription(
            String,
            '/scan_state',
            self._state_callback,
            10
        )

        # Publisher for gated scan
        self.scan_slam_pub = self.create_publisher(LaserScan, '/scan_slam', 10)

        # State tracking (default to MOVING/flat so it maps initially)
        self.current_state = "MOVING"

        self.get_logger().info("Scan Mux Node initialized. Default state: MOVING (forwarding scans).")

    def _state_callback(self, msg):
        state = msg.data.upper()
        if state != self.current_state:
            self.current_state = state
            self.get_logger().info(f"State changed to: {self.current_state}")
            if self.current_state in GATED_STATES:
                self.get_logger().info("TF frozen. Gating (blinding) 2D SLAM + odometry scan inputs.")
            else:
                self.get_logger().info("LiDAR is flat and tracking. Forwarding scans to 2D SLAM.")

    def _scan_callback(self, msg):
        # Forward scans only while the 2D pipeline is allowed to move the pose estimate
        if self.current_state not in GATED_STATES:
            self.scan_slam_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMuxNode()
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
