#!/usr/bin/env python3
"""
Scan to PointCloud2 Converter & Map Accumulator — FYDP Cv2 (v7.2)
=================================================================
Converts 2D LaserScan messages into 3D PointCloud2 using dynamic TF transforms,
filters points against Cartographer's 2D OccupancyGrid floorplan, and calculates
an axis-specific color palette blend (Z: Blue->Green, X: Red->Purple, Y: Orange->Yellow)
for maximum visual clarity in RViz.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from laser_geometry import LaserProjection
import tf2_sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os
import time


class ScanToPointCloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')

        self.declare_parameter('target_frame', 'map')
        self.target_frame = self.get_parameter('target_frame').value

        # Parameters for point cloud accumulation & filtering
        self.declare_parameter('voxel_size', 0.02)
        self.declare_parameter('invert_z', False)
        self.declare_parameter('filter_by_occupancy', True)
        self.declare_parameter('max_occupancy_threshold', 50)  # <= 50 is free space
        self.declare_parameter('save_dir', os.path.expanduser('~/fydp_maps'))

        self.voxel_size = self.get_parameter('voxel_size').value
        self.invert_z = self.get_parameter('invert_z').value
        self.filter_by_occupancy = self.get_parameter('filter_by_occupancy').value
        self.max_occupancy_threshold = self.get_parameter('max_occupancy_threshold').value
        self.save_dir = self.get_parameter('save_dir').value

        # Occupancy map data buffer
        self.occupancy_grid = None

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Laser projection utility
        self.laser_projector = LaserProjection()

        # Storage for voxel-filtered accumulated map points
        # key: (vx, vy, vz), value: [x, y, z, rgb_packed, intensity]
        self.map_points = {}
        self.scan_count = 0

        # Subscriptions
        scan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, scan_qos)
        self.state_sub = self.create_subscription(
            String, '/scan_state', self.state_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.occupancy_callback, 10)

        # Publishers
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/pointcloud_3d', 10)
        self.map_pub = self.create_publisher(
            PointCloud2, '/map_3d', 10)

        # State tracking
        self.current_state = "MOVING"

        # Services
        self.clear_service = self.create_service(
            Trigger, '/clear_map', self.clear_map_callback)
        self.save_service = self.create_service(
            Trigger, '/save_map', self.save_map_callback)

        # Define PointCloud2 fields (x, y, z, rgb, intensity)
        self.map_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
        ]

        # Timer to publish 3D map at steady 1 Hz
        self.map_timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info('ScanToPointCloud node started (v7.2 Axis-Specific Color Palettes)')
        self.get_logger().info(f'Target frame: {self.target_frame} | Voxel: {self.voxel_size}m')

    def occupancy_callback(self, msg):
        """Buffer latest 2D OccupancyGrid from Cartographer."""
        self.occupancy_grid = msg

    def clear_map_callback(self, request, response):
        self.map_points = {}
        response.success = True
        response.message = "Accumulated 3D map cleared."
        return response

    def save_map_callback(self, request, response):
        """Write the accumulated map to an ASCII .pcd file."""
        if not self.map_points:
            response.success = False
            response.message = "Nothing to save — the 3D map is empty."
            return response

        points = list(self.map_points.values())
        try:
            os.makedirs(self.save_dir, exist_ok=True)
            path = os.path.join(
                self.save_dir, time.strftime('fydp_map_%Y%m%d_%H%M%S.pcd'))
            with open(path, 'w') as f:
                f.write('# .PCD v0.7 - Point Cloud Data file format\n')
                f.write('VERSION 0.7\n')
                f.write('FIELDS x y z intensity\n')
                f.write('SIZE 4 4 4 4\n')
                f.write('TYPE F F F F\n')
                f.write('COUNT 1 1 1 1\n')
                f.write(f'WIDTH {len(points)}\n')
                f.write('HEIGHT 1\n')
                f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                f.write(f'POINTS {len(points)}\n')
                f.write('DATA ascii\n')
                for p in points:
                    f.write(f'{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {p[4]:.2f}\n')
        except OSError as e:
            response.success = False
            response.message = f"Failed to write map: {e}"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f"Saved {len(points)} points to {path}"
        self.get_logger().info(response.message)
        return response

    def state_callback(self, msg):
        self.current_state = msg.data.upper()

    def _lookup(self, frame_id, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, frame_id, stamp)
        except Exception:
            pass
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(
                f'No transform {frame_id} -> {self.target_frame}: {e}',
                throttle_duration_sec=2.0)
            return None

    def _filter_points_by_occupancy(self, pts):
        """Filter 3D points to keep only those within mapped 2D floorplan space."""
        if not self.filter_by_occupancy or self.occupancy_grid is None:
            return pts

        grid = self.occupancy_grid
        info = grid.info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution
        width = info.width
        height = info.height
        data = np.array(grid.data, dtype=np.int8)

        gx = np.floor((pts[:, 0] - origin_x) / res).astype(np.int32)
        gy = np.floor((pts[:, 1] - origin_y) / res).astype(np.int32)

        valid_bounds = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
        valid_indices = np.where(valid_bounds)[0]
        if len(valid_indices) == 0:
            return np.empty((0, pts.shape[1]), dtype=np.float32)

        cell_idx = gy[valid_indices] * width + gx[valid_indices]
        cell_vals = data[cell_idx]

        free_mask = (cell_vals >= 0) & (cell_vals <= self.max_occupancy_threshold)
        accepted_indices = valid_indices[free_mask]

        return pts[accepted_indices]

    def _compute_axis_specific_colors(self, pts):
        """Compute RGB colors based on your exact axis palette specifications:
        - Z-Axis (Height): Blue (0,0,255) -> Green (0,255,0)
        - X-Axis (Depth): Red (255,0,0) -> Purple (128,0,128)
        - Y-Axis (Lateral): Orange (255,140,0) -> Yellow (255,255,0)
        """
        if pts.shape[0] == 0:
            return np.empty((0, 5), dtype=np.float32)

        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

        # Normalized factor along Z-axis (height: -1.0m to +1.0m)
        t_z = np.clip((z + 0.5) / 1.5, 0.0, 1.0)
        # Z-color: Blue (0, 0, 255) -> Green (0, 255, 0)
        z_r = np.zeros_like(z)
        z_g = 255.0 * t_z
        z_b = 255.0 * (1.0 - t_z)

        # Normalized factor along X-axis (depth/forward)
        t_x = 0.5 + 0.5 * np.sin(x * 0.8)
        # X-color: Red (255, 0, 0) -> Purple (128, 0, 128)
        x_r = 255.0 - 127.0 * t_x
        x_g = np.zeros_like(x)
        x_b = 128.0 * t_x

        # Normalized factor along Y-axis (lateral/side)
        t_y = 0.5 + 0.5 * np.sin(y * 0.8)
        # Y-color: Orange (255, 140, 0) -> Yellow (255, 255, 0)
        y_r = np.full_like(y, 255.0)
        y_g = 140.0 + 115.0 * t_y
        y_b = np.zeros_like(y)

        # Blend colors based on spatial dominance (Z height vs XY position)
        blend_z = 0.5
        blend_xy = 0.5

        r_final = np.clip(blend_z * z_r + blend_xy * (0.5 * x_r + 0.5 * y_r), 0, 255).astype(np.uint32)
        g_final = np.clip(blend_z * z_g + blend_xy * (0.5 * x_g + 0.5 * y_g), 0, 255).astype(np.uint32)
        b_final = np.clip(blend_z * z_b + blend_xy * (0.5 * x_b + 0.5 * y_b), 0, 255).astype(np.uint32)

        # Pack RGB into float32 (standard ROS PointCloud2 format)
        rgb_packed = (r_final << 16) | (g_final << 8) | b_final
        rgb_float = rgb_packed.view(np.float32)

        out = np.zeros((pts.shape[0], 5), dtype=np.float32)
        out[:, 0] = x
        out[:, 1] = y
        out[:, 2] = z
        out[:, 3] = rgb_float
        out[:, 4] = z  # intensity field

        return out

    def scan_callback(self, scan_msg):
        try:
            cloud_in_laser_frame = self.laser_projector.projectLaser(scan_msg)
            transform = self._lookup(scan_msg.header.frame_id, scan_msg.header.stamp)
            if transform is None:
                return

            cloud_in_target_frame = tf2_sensor_msgs.do_transform_cloud(
                cloud_in_laser_frame, transform)

            available = {f.name for f in cloud_in_target_frame.fields}
            has_intensity = 'intensity' in available
            names = ['x', 'y', 'z', 'intensity'] if has_intensity else ['x', 'y', 'z']

            raw = pc2.read_points_numpy(
                cloud_in_target_frame, field_names=names, skip_nans=True)
            if raw.size == 0:
                return

            pts = np.zeros((raw.shape[0], 3), dtype=np.float32)
            pts[:, :3] = raw[:, :3]
            if self.invert_z:
                pts[:, 2] *= -1.0

            # 1. Filter points against 2D OccupancyGrid floorplan
            filtered_pts = self._filter_points_by_occupancy(pts)
            if filtered_pts.shape[0] == 0:
                return

            # 2. Compute axis-specific RGB colors (Z: Blue->Green, X: Red->Purple, Y: Orange->Yellow)
            colored_pts = self._compute_axis_specific_colors(filtered_pts)

            # Live single-scan slice
            self.cloud_pub.publish(self._pack_cloud(colored_pts))

            # 3. Accumulate points into 3D map during SCANNING state
            if self.current_state == "SCANNING":
                keys = np.floor(colored_pts[:, :3] / self.voxel_size).astype(np.int32)
                for key, p in zip(map(tuple, keys.tolist()), colored_pts):
                    self.map_points[key] = p

            self.scan_count += 1

        except Exception as e:
            self.get_logger().warn(
                f'Could not transform/accumulate scan: {e}', throttle_duration_sec=2.0)

    def _pack_cloud(self, pts):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = self.map_fields
        msg.is_bigendian = False
        msg.point_step = 20  # 5 float32 fields = 20 bytes
        msg.row_step = 20 * msg.width
        msg.is_dense = True
        msg.data = np.ascontiguousarray(pts, dtype=np.float32).tobytes()
        return msg

    def publish_map(self):
        if not self.map_points:
            return
        try:
            self.map_pub.publish(
                self._pack_cloud(np.array(list(self.map_points.values()),
                                          dtype=np.float32)))
        except Exception as e:
            self.get_logger().error(f"Failed to publish accumulated map: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointCloud()
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
