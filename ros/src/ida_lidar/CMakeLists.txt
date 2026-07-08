cmake_minimum_required(VERSION 3.8)
project(ida_lidar)

# Default to C++17 (supported by ROS 2 Humble on Ubuntu 22.04 / GCC 11)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(std_srvs REQUIRED)

# 2. Find Point Cloud Library (PCL)
find_package(PCL 1.12 REQUIRED)

# 3. Include Directories
# This tells the compiler to look in your new 'include' folder AND the PCL folders
include_directories(
  include
  third_party/rplidar_sdk/include
  third_party/rplidar_sdk/src
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 4. Compile the executable
add_executable(buoy_lidar
  src/BuoyLidarNode.cpp
  src/BuoyDetector.cpp
)

# 5. Link ROS 2 and PCL dependencies
ament_target_dependencies(buoy_lidar
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  tf2_ros
  tf2_geometry_msgs
  laser_geometry
)
target_link_libraries(buoy_lidar ${PCL_LIBRARIES})

# GPS/IMU odometry node — publishes odom → base_link transform from sensor data
add_executable(gps_imu_odom
  src/GpsImuOdometry.cpp
)
ament_target_dependencies(gps_imu_odom
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

# RPLIDAR C1 driver — talks to the sensor directly over serial via the vendored
# SDK (third_party/rplidar_sdk), replacing the external rplidar_ros dependency
# so this package owns motor start/stop.
file(GLOB RPLIDAR_SDK_SRC
  "third_party/rplidar_sdk/src/*.cpp"
  "third_party/rplidar_sdk/src/arch/linux/*.cpp"
  "third_party/rplidar_sdk/src/dataunpacker/*.cpp"
  "third_party/rplidar_sdk/src/dataunpacker/unpacker/*.cpp"
  "third_party/rplidar_sdk/src/hal/*.cpp"
)
add_executable(rplidar_driver
  src/RplidarNode.cpp
  ${RPLIDAR_SDK_SRC}
)
ament_target_dependencies(rplidar_driver
  rclcpp
  sensor_msgs
  std_srvs
)
target_link_libraries(rplidar_driver pthread)

# 6. Install executables
install(TARGETS buoy_lidar gps_imu_odom rplidar_driver
  DESTINATION lib/${PROJECT_NAME}
)

# 7. Install the Include Directory (Required for the ideal structure)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
