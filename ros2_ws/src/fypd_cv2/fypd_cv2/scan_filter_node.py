#!/usr/bin/env python3
"""Scan Filter Node — filters tilted scans from reaching slam_toolbox."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter_node')

        # Current state — default to forwarding (assume flat)
        self.current_state = ''

        # Subscribers
        self.scan_state_sub = self.create_subscription(
            String, '/scan_state', self.state_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.scan_filtered_pub = self.create_publisher(
            LaserScan, '/scan_filtered', 10)

        self.get_logger().info('Scan filter node started — forwarding scans when not SCANNING')

    def state_callback(self, msg):
        """Update current ESP32 state machine state."""
        self.current_state = msg.data
        self.get_logger().debug(f'Scan state updated: {self.current_state}')

    def scan_callback(self, msg):
        """Forward scan to /scan_filtered only if state is not SCANNING."""
        if self.current_state != 'SCANNING':
            self.scan_filtered_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
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
