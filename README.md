# IDA LiDAR Perception (MAVİ İNCİ) — RPLIDAR C1 Branch

**A ROS 2 Humble perception stack for autonomous Unmanned Surface Vehicles (USVs).**

This branch adapts the pipeline for the **RPLIDAR C1**, a 2D single-plane LiDAR. The sensor publishes `sensor_msgs/LaserScan` on `/scan` instead of a 3D point cloud. The detection pipeline is rewritten accordingly: RANSAC water surface removal is replaced by intensity thresholding and arc size validation, and a temporal persistence filter is added to suppress wave noise and pitch-induced false returns.

For the 3D LiDAR version of this package, see the `main` branch.

## Key Features
* **RPLIDAR C1 Support:** Subscribes to `sensor_msgs/LaserScan`, converts to PCL via `laser_geometry`.
* **Intensity Filtering:** Drops weak returns (water glints, spray) before clustering — buoys return significantly stronger signals than water.
* **Arc Size Validation:** Each cluster's physical chord length is computed at its measured range. Clusters outside the expected buoy width (0.1–1.5 m) are rejected.
* **Temporal Persistence Filter:** A detection must appear in at least 3 of the last 5 frames before being reported — eliminates single-frame wave noise and brief pitch-induced disappearances.
* **GPS/IMU Odometry:** Fuses `NavSatFix` and `Imu` data to broadcast a drift-corrected `odom → base_link` transform in real time.
* **RViz2 Visualisation:** Colour-coded markers (green < 5 m / yellow < 10 m / red ≥ 10 m) with floating distance labels.

## Detection Pipeline

```
LaserScan (/scan)
  └── laser_geometry projection  →  PointCloud2 with intensity
  └── intensity filter           →  drop points below threshold (water glints)
  └── ROI passthrough            →  keep ±10 m box (C1 reliable range)
  └── voxel grid downsample      →  0.05 m leaf size
  └── Euclidean clustering       →  tolerance 0.3 m, min 3 pts, max 50 pts
  └── arc size validation        →  chord length 0.1–1.5 m at measured range
  └── TF2 transform              →  local lidar_link → global odom frame
  └── temporal persistence       →  confirm after 3 of 5 consecutive frames
  └── MarkerArray (/buoy_markers)
```

## Prerequisites

```bash
sudo apt update
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
                 ros-humble-geometry-msgs ros-humble-ros-gz \
                 ros-humble-laser-geometry libpcl-dev
```

## Installation & Build

The `ros/` folder in this repo *is* the colcon workspace — there's no separate
workspace to set up elsewhere.

```bash
# 1. Clone the repo
git clone -b rplidarc1 <repo-url> Lidar
cd Lidar/ros

# 2. Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build --packages-select ida_lidar

# 4. Source
source install/setup.bash
```

## Usage

### Manual simulation pipeline

**1. Launch Gazebo:**

```bash
# from Lidar/ros (the workspace root) — gazebo/ lives one level up, at the repo root
cd ..
gz sim -v 4 gazebo/model.sdf
```

> **Note — GPS spherical coordinates:** For the GPS sensor to produce realistic
> latitude/longitude values, your world SDF must include a `<spherical_coordinates>`
> block set to your test location. Without it, Gazebo defaults to (0°, 0°).
> ```xml
> <spherical_coordinates>
>   <surface_model>EARTH_WGS84</surface_model>
>   <latitude_deg>YOUR_LAT</latitude_deg>
>   <longitude_deg>YOUR_LON</longitude_deg>
>   <elevation>0</elevation>
> </spherical_coordinates>
> ```

**2. Start the ROS–Gazebo bridge:**

The RPLIDAR C1 simulation publishes `LaserScan`, not `PointCloud2`.

```bash
ros2 run ros_gz_bridge parameter_bridge \
    /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
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

### Physical hardware (real RPLIDAR C1)

Install the RPLIDAR ROS 2 driver and launch it before starting the nodes:

```bash
sudo apt install ros-humble-rplidar-ros
ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/ttyUSB0 -p frame_id:=lidar_link
```

Then run `gps_imu_odom` and `buoy_lidar` as above.

## Node Details

### `buoy_lidar`
| | |
|---|---|
| Subscribes | `/scan` (`sensor_msgs/LaserScan`) |
| Publishes | `/buoy_markers` (`visualization_msgs/MarkerArray`) |
| Requires TF | `odom → base_link → lidar_link` |

### `gps_imu_odom`
| | |
|---|---|
| Subscribes | `/gps/fix` (`sensor_msgs/NavSatFix`), `/imu/data` (`sensor_msgs/Imu`) |
| Publishes | `/odom` (`nav_msgs/Odometry`) |
| Broadcasts TF | `odom → base_link` |

The first valid GPS fix is latched as the local coordinate origin. Subsequent fixes are projected to local X/Y metres using equirectangular approximation (accurate to ~0.1% under 50 km). Orientation is taken from the IMU's onboard fusion output.

## Tunable Parameters

All constants are at the top of their respective source files.

| Parameter | File | Default | Effect |
|---|---|---|---|
| `MIN_INTENSITY` | `BuoyLidarNode.cpp` | `50.0` | Raise to reject more water noise; lower if buoys are missed |
| `PERSISTENCE_WINDOW` | `BuoyLidarNode.cpp` | `5` | Frames in the sliding window |
| `PERSISTENCE_MIN_HITS` | `BuoyLidarNode.cpp` | `3` | Min frames a detection must appear in to be confirmed |
| `PERSISTENCE_RADIUS` | `BuoyLidarNode.cpp` | `1.0 m` | Max displacement between frames to count as the same buoy |
| `MIN_BUOY_ARC_M` | `BuoyDetector.hpp` | `0.1 m` | Minimum physical cluster width |
| `MAX_BUOY_ARC_M` | `BuoyDetector.hpp` | `1.5 m` | Maximum physical cluster width |
| `ROI_X/Y` | `BuoyDetector.cpp` | `±10 m` | Detection range box |

## TF Frame Chain

```
odom
 └── base_link      ← broadcast by gps_imu_odom (GPS position + IMU orientation)
      └── lidar_link ← fixed joint in model.sdf (0.5 m forward, 0.5 m above hull)
```

## File Structure

```
ida_lidar/
├── CMakeLists.txt
├── package.xml
├── include/ida_lidar/
│   ├── BuoyDetector.hpp       # PCL pipeline interface + arc filter constants
│   ├── BuoyLidarNode.hpp      # buoy_lidar node interface
│   └── GpsImuOdometry.hpp     # gps_imu_odom node interface
├── src/
│   ├── BuoyLidarNode.cpp      # buoy_lidar node — intensity filter, persistence, markers, main()
│   ├── BuoyDetector.cpp       # ROI, voxel grid, clustering, arc validation
│   └── GpsImuOdometry.cpp     # gps_imu_odom node — equirectangular projection, TF broadcast, main()
```

Each ROS 2 node is a `.hpp`/`.cpp` pair under `include/ida_lidar/` and `src/`, same as any other class — no file holds more than one node or utility, and each `.cpp` is buildable/readable on its own.

## Contributing

PCL algorithm changes in `BuoyDetector.cpp` must be profiled on the target hardware (Jetson Nano) before committing — this node runs in real time at 10 Hz during water trials. The `MIN_INTENSITY` threshold and arc size bounds should be re-calibrated against actual competition buoys before each event, as retroreflectivity and size vary between competitions.

---

<div align="center">

💙 **Bu Repo IEEE Ege Mavi İnci İnsansız Deniz Aracı Takımı Yazılım Ekibi Tarafından Oluşturulmuştur, Yazılım Ekibimize Sevgilerle**

[@NightKnight-nx2](https://github.com/NightKnight-nx2) · [@yalinoner](https://github.com/yalinoner) · [@nilayyldz](https://github.com/nilayyldz)

</div>
