import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('fypd_cv2')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument(
            'invert_z', default_value='False',
            description='Invert Z axis to fix upside-down height map'),
        DeclareLaunchArgument(
            'settle_window_sec', default_value='3.0',
            description='Seconds of settled 2D scans averaged when latching the frozen pose'),
        DeclareLaunchArgument(
            'voxel_size', default_value='0.02',
            description='3D map voxel resolution in metres'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz2 with the preconfigured FYDP display set'),

        # ── 1. LaserScan ➔ PointCloud2 Converter & Map Accumulator ──
        Node(
            package='fypd_cv2',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{
                'target_frame': 'map',
                'invert_z': ParameterValue(
                    LaunchConfiguration('invert_z'), value_type=bool),
                'voxel_size': ParameterValue(
                    LaunchConfiguration('voxel_size'), value_type=float),
            }],
            output='screen',
        ),

        # ── 2. Scan Multiplexer (gating scans during tilts) ──
        Node(
            package='fypd_cv2',
            executable='scan_mux_node',
            name='scan_mux_node',
            output='screen',
        ),

        # ── 2b. RF2O Laser Odometry (2D position tracking from scan matching) ──
        # publish_tf is FALSE on purpose — odom_gate_node owns odom -> base_link so the
        # transform can be frozen while the IMU says the table is standing still.
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan_slam',
                'odom_topic': '/odom_rf2o',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0,
            }],
        ),

        # ── 2c. Odometry Gate (IMU-gated owner of odom ➔ base_link) ──
        Node(
            package='fypd_cv2',
            executable='odom_gate_node',
            name='odom_gate_node',
            output='screen',
            parameters=[{
                'odom_in_topic': '/odom_rf2o',
                'odom_out_topic': '/odom',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_rate': 50.0,
                'settle_window_sec': ParameterValue(
                    LaunchConfiguration('settle_window_sec'), value_type=float),
                'freeze_states': ['SCANNING', 'SCAN_COMPLETE'],
            }],
        ),

        # ── 3. Google Cartographer 2D SLAM ──
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_2d.lua'
            ],
            remappings=[
                ('/scan', '/scan_slam'),
            ],
        ),

        # ── 4. Cartographer Occupancy Grid Node (2D Floorplan publisher) ──
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'resolution': 0.05,
                'publish_period_sec': 1.0,
            }],
        ),

        # ── 5. RViz2 (optional, rviz:=false to skip) ──
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'fydp_cv2.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='log',
        ),
    ])
