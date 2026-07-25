#!/usr/bin/env python3
"""GICP Registrator — aligns 3D sweeps using Open3D GICP."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
import open3d as o3d
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import struct


def pointcloud2_to_open3d(msg):
    """Convert a ROS2 PointCloud2 message to an Open3D PointCloud."""
    points = []
    for p in pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True):
        points.append([p[0], p[1], p[2]])

    pcd = o3d.geometry.PointCloud()
    if points:
        pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float64))
    return pcd


def open3d_to_pointcloud2(pcd, header):
    """Convert an Open3D PointCloud to a ROS2 PointCloud2 message."""
    points_np = np.asarray(pcd.points, dtype=np.float32)

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    # Build binary data
    point_step = 12  # 3 x float32
    data = bytearray()
    for pt in points_np:
        data.extend(struct.pack('fff', pt[0], pt[1], pt[2]))

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points_np)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * len(points_np)
    msg.data = bytes(data)
    msg.is_dense = True
    return msg


class GICPRegistrator(Node):
    def __init__(self):
        super().__init__('gicp_registrator')

        # Accumulated global map as Open3D PointCloud
        self.global_map = None
        self.sweep_count = 0

        # GICP parameters
        self.voxel_size = 0.03
        self.max_correspondence_distance = 0.5
        self.fitness_threshold = 0.3

        # Subscribe to completed sweep point clouds
        self.sweep_sub = self.create_subscription(
            PointCloud2, '/sweep_3d', self.sweep_callback, 10)

        # Subscribe to scan state (for logging)
        self.state_sub = self.create_subscription(
            String, '/scan_state', self.state_callback, 10)
        self.scan_state = ''

        # Publish the globally registered map
        self.map_pub = self.create_publisher(
            PointCloud2, '/map_3d_registered', 10)

        self.get_logger().info('GICPRegistrator node started')
        self.get_logger().info(
            f'Voxel size: {self.voxel_size}m, '
            f'Max correspondence dist: {self.max_correspondence_distance}m, '
            f'Fitness threshold: {self.fitness_threshold}')

    def state_callback(self, msg):
        self.scan_state = msg.data.strip()

    def sweep_callback(self, msg):
        self.sweep_count += 1
        self.get_logger().info(
            f'Received sweep #{self.sweep_count} ({msg.width * msg.height} points)')

        # 1. Convert PointCloud2 → Open3D PointCloud
        sweep_pcd = pointcloud2_to_open3d(msg)
        num_raw = len(sweep_pcd.points)

        if num_raw == 0:
            self.get_logger().warn('Received empty sweep, skipping.')
            return

        # 2. Voxel downsample the incoming sweep
        sweep_pcd = sweep_pcd.voxel_down_sample(voxel_size=self.voxel_size)
        num_downsampled = len(sweep_pcd.points)
        self.get_logger().info(
            f'Sweep #{self.sweep_count}: {num_raw} raw → {num_downsampled} downsampled points')

        # 3. First sweep — just store as the global map
        if self.global_map is None:
            self.global_map = sweep_pcd
            self.get_logger().info(
                f'First sweep stored as global map ({num_downsampled} points)')
            self.publish_map(msg.header)
            return

        # 4. Subsequent sweeps — register with GICP
        # Estimate normals (required by GICP)
        sweep_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 4.0, max_nn=30))
        self.global_map.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 4.0, max_nn=30))

        # GICP registration: initial transform = identity (slam_toolbox already aligned)
        init_transform = np.eye(4)

        reg_result = o3d.pipelines.registration.registration_generalized_icp(
            source=sweep_pcd,
            target=self.global_map,
            max_correspondence_distance=self.max_correspondence_distance,
            init=init_transform,
            estimation_method=o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=50
            )
        )

        fitness = reg_result.fitness
        rmse = reg_result.inlier_rmse

        self.get_logger().info(
            f'GICP result — fitness: {fitness:.4f}, RMSE: {rmse:.6f}')

        # 5. Accept or reject the registration
        if fitness > self.fitness_threshold:
            # Good registration — transform the sweep
            sweep_pcd.transform(reg_result.transformation)
            self.get_logger().info(
                f'Sweep #{self.sweep_count}: GICP accepted (fitness {fitness:.4f} > {self.fitness_threshold})')
        else:
            # Poor registration — trust slam_toolbox pose as-is (identity)
            self.get_logger().warn(
                f'Sweep #{self.sweep_count}: GICP rejected (fitness {fitness:.4f} < {self.fitness_threshold}), '
                f'using slam_toolbox pose')

        # 6. Merge sweep into global map
        self.global_map = self.global_map + sweep_pcd

        # 7. Downsample the merged result to keep memory bounded
        self.global_map = self.global_map.voxel_down_sample(voxel_size=self.voxel_size)
        num_merged = len(self.global_map.points)
        self.get_logger().info(
            f'Global map now has {num_merged} points after merge + downsample')

        # 8. Publish the updated global map
        self.publish_map(msg.header)

    def publish_map(self, header):
        """Publish the global map as a PointCloud2 message."""
        if self.global_map is None or len(self.global_map.points) == 0:
            return

        try:
            map_msg = open3d_to_pointcloud2(self.global_map, header)
            self.map_pub.publish(map_msg)
            self.get_logger().info(
                f'Published /map_3d_registered ({len(self.global_map.points)} points)')
        except Exception as e:
            self.get_logger().error(f'Failed to publish registered map: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GICPRegistrator()
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
