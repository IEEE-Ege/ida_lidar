cmake_minimum_required(VERSION 3.8)
project(ida_lidar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# 2. Find Point Cloud Library (PCL)
find_package(PCL 1.12 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 3. Compile the executable
# This tells CMake to build 'buoy_lidar' using both C++ files
add_executable(buoy_lidar src/main.cpp src/BuoyDetector.cpp)

# 4. Link ROS 2 and PCL to the executable
ament_target_dependencies(buoy_lidar
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(buoy_lidar ${PCL_LIBRARIES})

# 5. The Install Rule (The crucial fix!)
# This moves the compiled 'buoy_lidar' into the folder where ROS 2 can find it
install(TARGETS buoy_lidar
  DESTINATION lib/${PROJECT_NAME})

ament_package()
