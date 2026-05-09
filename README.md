# IDA LiDAR Perception (MAVİ İNCİ)

**A ROS 2 Jazzy perception stack for autonomous Unmanned Surface Vehicles (USVs).**

This package contains the `buoy_lidar` node, an edge-optimized C++ point cloud processing pipeline designed for the MAVİ İNCİ autonomous boat. It ingests 3D LiDAR data, utilizes the Point Cloud Library (PCL) for obstacle extraction, and integrates ROS 2 `tf2` spatial transformations to map maritime buoys in global coordinates. A companion `gps_imu_odom` node fuses GPS and IMU data to continuously publish the `odom → base_link` transform, ensuring buoy positions remain accurate as the boat moves.

## Key Features
* **Native ROS 2 Integration:** Fully compatible with ROS 2 Jazzy Jalisco.
* **Gazebo Harmonic Ready:** Out-of-the-box support for simulated maritime environments and `ros_gz_bridge`.
* **Edge-Optimized PCL Pipeline:** Voxel Grid downsampling → RANSAC water surface removal → Euclidean clustering, designed for high framerates on embedded hardware (e.g., Jetson Nano).
* **GPS/IMU Odometry:** Fuses NavSatFix and IMU data using an equirectangular projection to publish a drift-corrected `odom → base_link` transform in real time.
* **RViz2 Visualisation:** Colour-coded buoy markers (green/yellow/red by range) with floating distance labels, pre-configured layout included.

## Prerequisites

```bash
sudo apt update
sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
                 ros-jazzy-geometry-msgs ros-jazzy-ros-gz libpcl-dev
```

## Installation & Build

```bash
# 1. Clone into your ROS 2 workspace
cd ~/mavi_inci_ws/src
git clone <repo-url> Lidar

# 2. Install dependencies
cd ~/mavi_inci_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build --packages-select ida_lidar

# 4. Source
source install/setup.bash
```

## Usage

### Bag file playback (testing / teammate onboarding)

```bash
ros2 launch ida_lidar playback.launch.py bag_path:=/path/to/your/bag
```

This single command starts the GPS/IMU odometry node, the perception node, bag playback (looped), and RViz2 with the pre-configured layout.

### Manual simulation pipeline

**1. Launch Gazebo:**

```bash
gz sim -v 4 gazebo/model.sdf
```

> **Note — GPS spherical coordinates:** For the GPS sensor in Gazebo to produce realistic latitude/longitude values, your world SDF must declare a `<spherical_coordinates>` block with the coordinates of your test location. Without it, Gazebo defaults to (0°, 0°) which is valid but places your origin in the Gulf of Guinea. Add this inside your `<world>` element:
> ```xml
> <spherical_coordinates>
>   <surface_model>EARTH_WGS84</surface_model>
>   <latitude_deg>YOUR_LAT</latitude_deg>
>   <longitude_deg>YOUR_LON</longitude_deg>
>   <elevation>0</elevation>
> </spherical_coordinates>
> ```

**2. Start the ROS–Gazebo bridge:**

```bash
ros2 run ros_gz_bridge parameter_bridge \
    /gazebo_lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
    /gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
    /imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU
```

**3. Run the odometry node:**

```bash
ros2 run ida_lidar gps_imu_odom
```

**4. Run the perception node:**

```bash
ros2 run ida_lidar buoy_lidar
```

## Node Details

### `buoy_lidar`
| | |
|---|---|
| Subscribes | `/gazebo_lidar/points` (`sensor_msgs/PointCloud2`) |
| Publishes | `/buoy_markers` (`visualization_msgs/MarkerArray`) |
| Requires TF | `odom → base_link → lidar_link` |

### `gps_imu_odom`
| | |
|---|---|
| Subscribes | `/gps/fix` (`sensor_msgs/NavSatFix`), `/imu/data` (`sensor_msgs/Imu`) |
| Publishes | `/odom` (`nav_msgs/Odometry`) |
| Broadcasts TF | `odom → base_link` |

The first valid GPS fix is latched as the local coordinate origin. All subsequent fixes are projected into local X/Y metres using an equirectangular approximation (accurate to ~0.1% for ranges under 50 km). Orientation is taken directly from the IMU's onboard fusion output.

## TF Frame Chain

```
odom
 └── base_link      ← broadcast by gps_imu_odom (GPS position + IMU orientation)
      └── lidar_link ← fixed joint defined in model.sdf
```

## File Structure

```
ida_lidar/
├── CMakeLists.txt
├── package.xml
├── include/ida_lidar/
│   ├── BuoyDetector.hpp       # PCL pipeline interface
│   └── GpsImuOdometry.hpp     # GPS/IMU odometry node interface
├── src/
│   ├── main.cpp               # buoy_lidar node — TF2, markers, detection loop
│   ├── BuoyDetector.cpp       # Voxel grid, RANSAC, Euclidean clustering
│   └── GpsImuOdometry.cpp     # gps_imu_odom node — equirectangular projection, TF broadcast
```

## Contributing

For the MAVİ İNCİ engineering team: PCL algorithm changes in `BuoyDetector.cpp` must be profiled for performance — this node runs in real time on edge compute during physical water trials. GPS/IMU parameters (noise, update rates) in `model.sdf` should be updated to match the actual hardware spec before deploying to the physical boat.
