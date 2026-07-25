#!/usr/bin/env python3
"""
Scan to PointCloud2 Converter & Map Accumulator — FYDP Cv2
=========================================================
Converts 2D LaserScan messages into 3D PointCloud2 using dynamic TF transforms,
and accumulates them into a persistent 3D voxel map only during scanning sweeps.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
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

        # Parameters for point cloud accumulation
        self.declare_parameter('voxel_size', 0.02)
        self.declare_parameter('invert_z', False)
        self.declare_parameter('save_dir', os.path.expanduser('~/fydp_maps'))

        self.voxel_size = self.get_parameter('voxel_size').value
        self.invert_z = self.get_parameter('invert_z').value
        self.save_dir = self.get_parameter('save_dir').value

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Laser projection utility
        self.laser_projector = LaserProjection()

        # Storage for voxel-filtered accumulated map points
        # key: (vx, vy, vz), value: [x, y, z, intensity]
        self.map_points = {}
        self.scan_count = 0

        # Subscriptions.
        # Depth 1 + best-effort on /scan is deliberate. Projecting and voxelising is
        # heavier than the 15 Hz scan rate on a bad frame, and with a deep queue the
        # node never recovers: it works through a backlog whose timestamps fall out of
        # the back of the TF cache, so every lookup fails and *nothing* is mapped.
        # Dropping a stale scan costs one slice of a 30 s sweep; falling behind costs
        # the whole sweep.
        scan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, scan_qos)
        self.state_sub = self.create_subscription(
            String, '/scan_state', self.state_callback, 10)

        # Publishers
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/pointcloud_3d', 10)
        self.map_pub = self.create_publisher(
            PointCloud2, '/map_3d', 10)

        # State tracking
        self.current_state = "MOVING"

        # Service to clear/delete the accumulated map
        self.clear_service = self.create_service(
            Trigger, '/clear_map', self.clear_map_callback)

        # Service to dump the accumulated map to a .pcd file on disk
        self.save_service = self.create_service(
            Trigger, '/save_map', self.save_map_callback)

        # Define PointCloud2 fields
        self.map_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Republish the accumulated map at a steady 1 Hz, independent of scan rate
        self.map_timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info('ScanToPointCloud node started')
        self.get_logger().info(f'Projecting scans into target frame: {self.target_frame}')
        self.get_logger().info(f'Voxel filter resolution set to: {self.voxel_size}m')

    def clear_map_callback(self, request, response):
        self.map_points = {}
        response.success = True
        response.message = "Accumulated 3D map cleared."
        return response

    def save_map_callback(self, request, response):
        """Write the accumulated map to an ASCII .pcd (readable by CloudCompare/MeshLab)."""
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
                for x, y, z, i in points:
                    f.write(f'{x:.4f} {y:.4f} {z:.4f} {i:.2f}\n')
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
        """Resolve frame_id -> target_frame, preferring the scan's own timestamp.

        Both lookups are non-blocking on purpose. This callback runs on a
        single-threaded executor, so a blocking tf timeout stalls the very
        executor that feeds /tf into the buffer: the node cannot catch up, the
        subscription queue backs up, and every subsequent scan is looked up
        further in the past until nothing resolves at all.
        """
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, frame_id, stamp)
        except Exception:
            pass
        # Fall back to the newest transform available. During a sweep the tilt
        # moves fast, so this is only worth doing to keep the live view alive.
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(
                f'No transform {frame_id} -> {self.target_frame}: {e}',
                throttle_duration_sec=2.0)
            return None

    def scan_callback(self, scan_msg):
        try:
            # 1. Project the LaserScan into a PointCloud2 in its own frame (e.g. 'laser')
            cloud_in_laser_frame = self.laser_projector.projectLaser(scan_msg)

            # 2. Lookup the transform from the laser frame to the target frame (typically 'map')
            transform = self._lookup(scan_msg.header.frame_id, scan_msg.header.stamp)
            if transform is None:
                return

            # 3. Transform the PointCloud2 into the target frame (resolving IMU + stepper tilt + SLAM pose)
            cloud_in_target_frame = tf2_sensor_msgs.do_transform_cloud(
                cloud_in_laser_frame, transform)

            # laser_geometry only emits an intensity field when the driver populated
            # scan.intensities. Ask for what is actually there, not what we hope for.
            available = {f.name for f in cloud_in_target_frame.fields}
            has_intensity = 'intensity' in available
            names = ['x', 'y', 'z', 'intensity'] if has_intensity else ['x', 'y', 'z']

            raw = pc2.read_points_numpy(
                cloud_in_target_frame, field_names=names, skip_nans=True)
            if raw.size == 0:
                return

            # Pack into a contiguous (N, 4) xyz+intensity block
            pts = np.zeros((raw.shape[0], 4), dtype=np.float32)
            pts[:, :3] = raw[:, :3]
            if has_intensity:
                pts[:, 3] = raw[:, 3]
            if self.invert_z:
                pts[:, 2] *= -1.0

            # Live single-scan slice for the real-time view
            self.cloud_pub.publish(self._pack_cloud(pts))

            # 4. Accumulate points into the 3D map ONLY during stationary active sweeps (SCANNING state)
            if self.current_state == "SCANNING":
                # floor, not int(): truncation toward zero makes a double-width
                # voxel straddling the origin on every axis.
                keys = np.floor(pts[:, :3] / self.voxel_size).astype(np.int32)
                for key, p in zip(map(tuple, keys.tolist()), pts):
                    self.map_points[key] = p

            self.scan_count += 1

        except Exception as e:
            self.get_logger().warn(
                f'Could not transform/accumulate scan: {e}', throttle_duration_sec=2.0)

    def _pack_cloud(self, pts):
        """Build a PointCloud2 straight from an (N, 4) float32 block.

        pc2.create_cloud walks the array in Python and struct-packs point by point,
        which is the single most expensive thing in this node once the map is large.
        The layout here is already exactly what the message wants, so one tobytes()
        does the whole buffer.
        """
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = self.map_fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * msg.width
        msg.is_dense = True
        msg.data = np.ascontiguousarray(pts, dtype=np.float32).tobytes()
        return msg

    def publish_map(self):
        """Publish the accumulated map on a timer, not per scan.

        A 30 s sweep accumulates well over 100k voxels. Re-serialising all of them
        inside the scan callback grows without bound and eventually starves the scan
        path, so this runs on its own timer.
        """
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
