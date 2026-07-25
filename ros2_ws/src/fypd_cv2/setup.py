from setuptools import find_packages, setup

package_name = 'fypd_cv2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/rover_launch.py',
            'launch/laptop_launch.py'
        ]),
        ('share/' + package_name + '/urdf', ['urdf/rover.urdf']),
        ('share/' + package_name + '/config', ['config/cartographer_2d.lua']),
        ('share/' + package_name + '/rviz', ['rviz/fydp_cv2.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jun',
    maintainer_email='user@todo.com',
    description='FYDP Cv2 DIY 3D LiDAR scanning system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tilt_tf_broadcaster = fypd_cv2.tilt_tf_broadcaster:main',
            'scan_to_pointcloud = fypd_cv2.scan_to_pointcloud:main',
            'scan_mux_node = fypd_cv2.scan_mux_node:main',
            'odom_gate_node = fypd_cv2.odom_gate_node:main',
        ],
    },
)
