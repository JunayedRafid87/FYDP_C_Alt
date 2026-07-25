# FYDP_C_Alt: DIY 3D LiDAR Scanner for Search & Rescue (Native Jazzy)

This repository contains the complete firmware, ROS 2 drivers, and map processing software for the **Search and Rescue (SAR) 3D LiDAR Scanner**. The system distributes compute workloads between an **ESP32-S3** microcontroller (motor/IMU drivers), a **Horizon Robotics RDK X5** single-board computer (rover sensor node), and a **Laptop** (base station mapping and visualization).

> [!NOTE]
> This is the **native ROS 2 Jazzy** version. It runs entirely on your laptop without Docker.
> The rover still runs ROS 2 Humble — cross-version communication is handled by Cyclone DDS.

---

## 🛠 Hardware Configuration & Wiring

### 1. ESP32-S3 to TMC2209 (Stepper Motor Driver)
* **GPIO 4** → TMC2209 **STEP**
* **GPIO 5** → TMC2209 **DIR**
* **GPIO 6** → TMC2209 **EN** (LOW = enabled, active sweep)

### 2. ESP32-S3 to Fermion BNO055 (9-Axis IMU)
* **GPIO 8** → BNO055 **SDA** (I2C Data)
* **GPIO 9** → BNO055 **SCL** (I2C Clock)
* **3.3V / GND** → BNO055 **VCC / GND**

---

## 💾 Firmware Setup (PlatformIO)

The firmware is located in `/esp32_stepper/` and manages:
1. Reading base quaternion orientation at 50 Hz.
2. Motion gating (detecting base movement via BNO055 gyroscope magnitude `gyro_mag > 8.0 deg/sec` **or** linear acceleration `acc_mag > 1.2 m/s²`).
3. Back-and-forth tilt sweeping using a NEMA-17 motor controlled via TMC2209.
4. Serial reporting format: `IMU:qw,qx,qy,qz`, `STEP:angle`, and `MOVING:0/1`.

### Flashing the Firmware:
1. Connect the ESP32-S3 to your laptop via USB.
2. Enter bootloader mode: Press and hold the **BOOT** button, press **EN/RST** once, and release **BOOT**.
3. Flash the code:
   ```bash
   cd esp32_stepper
   ~/.platformio/penv/bin/pio run --target upload --upload-port /dev/ttyACM0
   ```
4. Press the **EN/RST** button once to boot the microcontroller.

---

## 🌐 Network Setup (Cyclone DDS)

Since the RDK X5 runs **ROS 2 Humble** (Ubuntu 22.04) and the Laptop runs **ROS 2 Jazzy** (Ubuntu 24.04), they must use **Cyclone DDS** to communicate reliably over WiFi:

1. **Install Cyclone DDS on both machines:**
   * **RDK X5:** `sudo apt install ros-humble-rmw-cyclonedds-cpp`
   * **Laptop:** `sudo apt install ros-jazzy-rmw-cyclonedds-cpp`

2. **Add to `~/.bashrc` on both machines:**
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```
   *Then reload your terminals:* `source ~/.bashrc`

---

## 🔄 Scan Cycle & Motion Gating

The ESP32 owns the state machine; everything on the laptop reacts to the `/scan_state` topic.

| State | Entered when | Duration | Stepper | 2D SLAM | 3D accumulation | TF axis |
|---|---|---|---|---|---|---|
| `MOVING` | IMU reports motion | until still 1 s | flat, driver released | **running** | off | **tracking** |
| `STABILIZING` | 1 s with no motion | **7 s** | held flat | **running** — settles the position fix | off | **tracking** |
| `SCANNING` | stabilize elapsed | **30 s** | sweeping ±45° | blinded | **on** | **frozen** |
| `SCAN_COMPLETE` | sweep elapsed | until motion | flat, driver asleep | blinded | off | **frozen** |

Any motion detected by the BNO055 (gyro > 8 °/s **or** linear accel > 1.2 m/s²) aborts
back to `MOVING` from any state, which is the only thing that unfreezes the axis.

### TF Ownership

Exactly one node owns each edge — no two publishers ever fight over a transform:

```
map  ──────────────► odom                 cartographer_node
odom ──────────────► base_link            odom_gate_node   ← IMU-gated, freezable
base_link ─────────► base_link_stabilized hardware_interface (IMU roll/pitch, freezable)
base_link_stabilized ► laser              robot_state_publisher (stepper joint)
```

While the table is stationary the first three edges are all held constant, so only the
stepper joint moves. The 3D sweep is therefore rigid and the point cloud cannot smear.

### Why the odometry gate exists

RF2O estimates planar motion from consecutive laser scans, which is what tracks the
table while it rolls. But pure scan matching jitters by a few millimetres even against a
perfectly static room, and RF2O goes blind for ~37 s at a time because the mux withholds
scans during the sweep. So RF2O runs with `publish_tf:=false` and publishes to
`/odom_rf2o`; `odom_gate_node` owns the actual transform and:

* **tracks** RF2O while `MOVING` / `STABILIZING`, composing *relative* motion onto its own
  anchor pose (so RF2O is free to jump or restart without the TF ever discontinuing);
* **latches** on the `STABILIZING` ➔ `SCANNING` edge, snapping to the **average of the
  last 3 s** of settled 2D scans rather than whichever single scan landed last;
* **freezes** through `SCANNING` and `SCAN_COMPLETE`, republishing that pose at 50 Hz with
  fresh timestamps so transforms never expire;
* **rejects** RF2O jumps over 0.5 m / 60° in one step and re-baselines instead.

---

## 🚀 Running the System

Set `export ROS_DOMAIN_ID=42` and `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in
`~/.bashrc` on **both** machines first.

### Step 1: RDK X5 (Headless Terminal)
Connect the RPLiDAR C1 and ESP32-S3 to the RDK's USB ports, then:
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fypd_cv2 rover_launch.py \
    serial_port_esp32:=/dev/ttyACM0 \
    serial_port_lidar:=/dev/ttyUSB0
```
Starts the LiDAR at **15 Hz**, the hardware interface, and `robot_state_publisher`.

### Step 2: Laptop (Native Jazzy)
```bash
cd ~/Varsity/FYDP/FYDP_C_Alt/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fypd_cv2 laptop_launch.py
```
Starts the scan mux, RF2O, the odometry gate, Cartographer, the occupancy grid, the 3D
accumulator, and RViz2 with the preconfigured display set (`rviz:=false` to skip RViz).

Useful arguments: `voxel_size:=0.02`, `invert_z:=False`, `settle_window_sec:=3.0`.

### Step 3: Save the map
```bash
ros2 service call /save_map std_srvs/srv/Trigger    # writes ~/fydp_maps/fydp_map_*.pcd
ros2 service call /clear_map std_srvs/srv/Trigger   # start a fresh 3D map
```

### Building

The RDK only runs the sensor half, and `rf2o_laser_odometry` pulls in Eigen/Boost, so
skip it there:
```bash
# RDK X5
colcon build --symlink-install --packages-skip rf2o_laser_odometry
# Laptop (native Jazzy — no Docker needed)
colcon build --symlink-install
```

> [!IMPORTANT]
> **The two machines must agree on the clock.** Scans are stamped on the RDK and the TF
> chain is stamped on the laptop; Cartographer's `lookup_transform_timeout_sec` is 0.2 s,
> so more than ~0.2 s of clock skew makes every transform lookup fail and nothing maps at
> all. Run `sudo apt install chrony` on both and point the RDK at the laptop, or check
> with `timedatectl` on each.

---

## 📐 Advanced Scanning Methods

### 1. Gyro + Linear-Accel Motion Gating
The BNO055 runs in `IMUPLUS` mode so the linear-acceleration vector is already
gravity-compensated. Motion is declared on gyro magnitude > 8 °/s **or** linear accel >
1.2 m/s² — thresholds chosen to sit above stepper vibration and baseline sensor noise, so
the rig does not falsely abort a sweep.

### 2. Latched Position Fix
The 7 s `STABILIZING` window exists so the 2D pipeline gets a settled fix before the axis
is frozen. The gate averages that window (circular mean for yaw) instead of trusting a
single scan match, which removes the last few millimetres of scan-matching jitter from
the pose the entire 30 s sweep is projected against.

### 3. Voxel Accumulation
`scan_to_pointcloud` projects every scan through the full TF chain into the `map` frame
and keys the result into a 2 cm voxel dictionary, so re-scanning the same surface costs
no extra memory and produces no double-wall ghosting. Points are only committed while
the state is `SCANNING`, i.e. only while the frame they are projected into is frozen.

---

## 🧭 Topic Reference

| Topic | Type | Published by | Notes |
|---|---|---|---|
| `/scan` | `LaserScan` | rplidar_ros | raw, 15 Hz |
| `/scan_slam` | `LaserScan` | scan_mux_node | gated — silent during `SCANNING`/`SCAN_COMPLETE` |
| `/scan_state` | `String` | hardware_interface | `MOVING`/`STABILIZING`/`SCANNING`/`SCAN_COMPLETE` |
| `/imu/data` | `Imu` | hardware_interface | BNO055 quaternion |
| `/moving` | `Bool` | hardware_interface | raw motion flag |
| `/odom_rf2o` | `Odometry` | rf2o_laser_odometry | raw scan-matched odometry, **no TF** |
| `/odom` | `Odometry` | odom_gate_node | gated odometry; what Cartographer consumes |
| `/map` | `OccupancyGrid` | cartographer_occupancy_grid_node | 2D floor plan |
| `/pointcloud_3d` | `PointCloud2` | scan_to_pointcloud | live single-scan slice |
| `/map_3d` | `PointCloud2` | scan_to_pointcloud | accumulated 3D map |
| `/scan_status_marker` | `Marker` | odom_gate_node | RViz text: state + `FROZEN`/`TRACKING` |

---

## 🔧 Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `No transform laser -> map` repeating, nothing in `/map_3d` | Clock skew between RDK and laptop, or `robot_state_publisher` not running on the RDK. Check `ros2 run tf2_ros tf2_echo map laser`. |
| Axis drifts during the 30 s sweep | `/scan_state` is not reaching the laptop — check `ros2 topic echo /scan_state`. Without it both gates default to tracking. |
| Sweep aborts constantly | Motion thresholds too tight for your surface. Raise `gyro_mag` / `acc_mag` in `esp32_stepper/src/main.cpp`. |
| Axis never unfreezes after a scan | Thresholds too loose — the IMU is not seeing the table move. Lower the same two constants. |
| 3D map smears in one direction | `settle_window_sec` too short, or the table is still creeping during `STABILIZING`. |
| Two nodes fighting over `odom` | `publish_tf` must stay **false** on `rf2o_laser_odometry`; only `odom_gate_node` owns that edge. |
