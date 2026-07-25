import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Path to URDF model
    pkg_share = get_package_share_directory('fypd_cv2')
    urdf_file = os.path.join(pkg_share, 'urdf', 'rover.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument(
            'serial_port_lidar', default_value='/dev/ttyUSB0',
            description='Serial port for RPLiDAR C1'),
        DeclareLaunchArgument(
            'serial_port_esp32', default_value='/dev/ttyACM0',
            description='Serial port for ESP32-S3'),
        DeclareLaunchArgument(
            'invert_stepper', default_value='False',
            description='Invert stepper tilt direction'),

        # ── 1. RPLiDAR C1 Driver (at 15 Hz scan rate) ──
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('serial_port_lidar'),
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'DenseBoost',
                'scan_frequency': 15.0,
            }],
            output='screen',
        ),

        # ── 2. Robot State Publisher (TF base_link_stabilized ➔ laser) ──
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        ),

        # ── 3. Static TF: base_link ➔ thermal_camera_link ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0.05', '0', '0.10', '0', '0', '0', 'base_link', 'thermal_camera_link'],
        ),

        # ── 4. Hardware Interface Node (serial data and base_link ➔ base_link_stabilized TF) ──
        Node(
            package='fypd_cv2',
            executable='tilt_tf_broadcaster',  # maps to entrypoint for HardwareInterfaceNode
            name='hardware_interface',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port_esp32'),
                'baud_rate': 115200,
                'invert_stepper': ParameterValue(
                    LaunchConfiguration('invert_stepper'), value_type=bool),
            }],
            output='screen',
        ),
    ])
