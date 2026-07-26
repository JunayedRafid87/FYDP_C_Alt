# FYDP_C_Alt: Official Launch Prompts (v8)

> Quick reference guide for launching the Search & Rescue 3D LiDAR Scanner system on both the **RDK X5 Rover** and the **Laptop Base Station**.

---

## 🤖 1. RDK X5 (Rover Node)

Run on the **RDK X5** single-board computer (Ubuntu 22.04 / ROS 2 Humble):

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fypd_cv2 rover_launch.py serial_port_esp32:=/dev/ttyACM0 serial_port_lidar:=/dev/ttyUSB0
```

> **Note:** Starts the RPLiDAR C1 driver (15 Hz), BNO055 IMU hardware interface node, and `robot_state_publisher`.

---

## 💻 2. Laptop (Base Station & Visualization)

Run on your **Laptop** (Ubuntu 24.04 / ROS 2 Jazzy — **Native, No Docker**):

```bash
cd ~/Varsity/FYDP/FYDP_C_Alt/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fypd_cv2 laptop_launch.py
```

> **Note:** Starts the Scan Mux, RF2O Laser Odometry, Odometry Gate, Cartographer 2D SLAM, Occupancy Grid, 3D Pointcloud Accumulator (with 2D floorplan boundary filtering & per-axis RGB coloring), and RViz2.

---

## 💾 3. Save / Clear 3D Point Cloud Map

Execute on the **Laptop** while system is running:

```bash
# Save 3D map to ~/fydp_maps/ (ASCII .pcd format for CloudCompare / MeshLab)
ros2 service call /save_map std_srvs/srv/Trigger

# Clear the current 3D map buffer
ros2 service call /clear_map std_srvs/srv/Trigger
```
