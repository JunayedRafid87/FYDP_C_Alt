#!/usr/bin/env python3
"""
Hardware Interface Node — FYDP Cv2
==================================
Reads tilt angle, IMU orientation, and movement status from the ESP32 serial port.
- Broadcasts the dynamic TF: base_link ➔ base_link_stabilized (IMU pitch/roll, zero translation and yaw).
- Publishes /joint_states (sensor_msgs/JointState) containing the stepper position in radians for nema_pitch_joint.
- Publishes /imu/data (sensor_msgs/Imu) with the current BNO055 orientation.
- Publishes /moving (std_msgs/Bool) and /scan_state (std_msgs/String).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState, Imu
from tf2_ros import TransformBroadcaster
import serial
import math
import threading


# States in which the stabilized frame is held still. The ESP32 only leaves these when
# the BNO055 reports real motion, so the axis stays put until the table is picked up.
FREEZE_STATES = ("SCANNING", "SCAN_COMPLETE")


class HardwareInterfaceNode(Node):
    def __init__(self):
        super().__init__('hardware_interface')

        # Declare parameters with defaults
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('invert_stepper', False)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.invert_stepper = self.get_parameter('invert_stepper').value
        # State and TF freezing tracking
        self.current_state = "MOVING"
        self.frozen_roll = 0.0
        self.frozen_pitch = 0.0
        self.last_roll = 0.0
        self.last_pitch = 0.0

        # Buffered gyro (rad/s) and raw accelerometer (m/s², with gravity)
        # Updated from GYRO: and ACCEL: serial lines, consumed by _handle_imu
        self.gyro = (0.0, 0.0, 0.0)
        self.accel = (0.0, 0.0, 9.81)

        # TF broadcaster for base_link ➔ base_link_stabilized
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.moving_pub = self.create_publisher(Bool, '/moving', 10)
        self.state_pub = self.create_publisher(String, '/scan_state', 10)

        # Open serial connection to ESP32
        try:
            self.ser = serial.Serial()
            self.ser.port = port
            self.ser.baudrate = baud
            self.ser.timeout = 0.1
            self.ser.dtr = False   # Prevent ESP32-S3 reset on connect
            self.ser.rts = False
            self.ser.open()
            import time
            time.sleep(2)  # Wait for ESP32 to stabilize after port open
            self.ser.reset_input_buffer()  # Flush any boot messages
            self.get_logger().info(f'Connected to ESP32 on {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self.get_logger().error('Check: ls /dev/ttyACM* /dev/ttyUSB*')
            raise

        # Read serial in a background thread
        self.running = True
        self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.serial_thread.start()

    def _serial_reader(self):
        """Continuously read serial data and publish topics."""
        while self.running and rclpy.ok():
            try:
                if not self.ser.is_open:
                    continue
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith('IMU:'):
                    parts = line.split(':')[1].split(',')
                    qw = float(parts[0])
                    qx = float(parts[1])
                    qy = float(parts[2])
                    qz = float(parts[3])
                    
                    if not (math.isnan(qw) or math.isnan(qx) or math.isnan(qy) or math.isnan(qz)):
                        self._handle_imu(qw, qx, qy, qz)
                elif line.startswith('GYRO:'):
                    parts = line.split(':')[1].split(',')
                    self.gyro = (float(parts[0]), float(parts[1]), float(parts[2]))
                elif line.startswith('ACCEL:'):
                    parts = line.split(':')[1].split(',')
                    self.accel = (float(parts[0]), float(parts[1]), float(parts[2]))
                elif line.startswith('STEP:'):
                    angle_deg = float(line.split(':')[1])
                    self._handle_stepper(angle_deg)
                elif line.startswith('MOVING:'):
                    is_moving_val = int(line.split(':')[1])
                    msg = Bool()
                    msg.data = (is_moving_val == 1)
                    self.moving_pub.publish(msg)
                elif line.startswith('STATE:'):
                    state_str = line.split(':')[1]
                    was_frozen = self.current_state in FREEZE_STATES
                    now_frozen = state_str in FREEZE_STATES
                    if now_frozen and not was_frozen:
                        self.frozen_roll = self.last_roll
                        self.frozen_pitch = self.last_pitch
                        self.get_logger().info(
                            f"{state_str}: freezing base_link -> base_link_stabilized TF "
                            f"(Roll: {math.degrees(self.frozen_roll):.2f}°, Pitch: {math.degrees(self.frozen_pitch):.2f}°). "
                            f"Held until the IMU reports motion."
                        )
                    elif was_frozen and not now_frozen:
                        self.get_logger().info(
                            f"{state_str}: motion detected — base_link -> base_link_stabilized TF live again."
                        )
                    self.current_state = state_str
                    msg = String()
                    msg.data = state_str
                    self.state_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error in serial reader: {e}", throttle_duration_sec=5.0)

    def _handle_imu(self, qw, qx, qy, qz):
        """Publish /imu/data and broadcast base_link ➔ base_link_stabilized."""
        now_msg = self.get_clock().now().to_msg()

        # 1. Publish standard Imu message with full sensor data
        imu_msg = Imu()
        imu_msg.header.stamp = now_msg
        imu_msg.header.frame_id = 'base_link'

        imu_msg.orientation.w = qw
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz

        # Angular velocity from BNO055 gyroscope (rad/s)
        imu_msg.angular_velocity.x = self.gyro[0]
        imu_msg.angular_velocity.y = self.gyro[1]
        imu_msg.angular_velocity.z = self.gyro[2]

        # Raw linear acceleration WITH gravity (m/s²) — Cartographer needs
        # gravity present so it can determine which way is "up"
        imu_msg.linear_acceleration.x = self.accel[0]
        imu_msg.linear_acceleration.y = self.accel[1]
        imu_msg.linear_acceleration.z = self.accel[2]

        # Covariance matrices (diagonal, small values = high confidence)
        # BNO055 orientation is pre-fused on-chip
        imu_msg.orientation_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001]
        # BNO055 gyroscope noise ~0.01 rad/s
        imu_msg.angular_velocity_covariance = [
            0.0001, 0.0, 0.0,
            0.0, 0.0001, 0.0,
            0.0, 0.0, 0.0001]
        # BNO055 accelerometer noise ~0.06 m/s²
        imu_msg.linear_acceleration_covariance = [
            0.004, 0.0, 0.0,
            0.0, 0.004, 0.0,
            0.0, 0.0, 0.004]

        self.imu_pub.publish(imu_msg)

        # 2. Extract roll and pitch
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        self.last_roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            self.last_pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            self.last_pitch = math.asin(sinp)

        # While frozen, hold the latched orientation so the base_link_stabilized axis is
        # perfectly still through the 3D sweep and afterwards, until the IMU sees motion.
        if self.current_state in FREEZE_STATES:
            roll = self.frozen_roll
            pitch = self.frozen_pitch
        else:
            roll = self.last_roll
            pitch = self.last_pitch

        # Rebuild quaternion with yaw = 0
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cr = math.cos(roll / 2.0)
        sr = math.sin(roll / 2.0)

        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'base_link_stabilized'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.w = cr * cp
        t.transform.rotation.x = sr * cp
        t.transform.rotation.y = cr * sp
        t.transform.rotation.z = -sr * sp

        self.tf_broadcaster.sendTransform(t)

    def _handle_stepper(self, angle_deg):
        """Publish /joint_states containing the nema_pitch_joint position in radians."""
        if self.invert_stepper:
            angle_deg = -angle_deg

        angle_rad = math.radians(angle_deg)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['nema_pitch_joint']
        js.position = [angle_rad]
        js.velocity = []
        js.effort = []

        self.joint_state_pub.publish(js)

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterfaceNode()
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
