cmake_minimum_required(VERSION 3.8)
project(ida_lidar)

# Default to C++17 (Standard for modern ROS 2 Jazzy)
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

# 2. Find Point Cloud Library (PCL)
find_package(PCL 1.12 REQUIRED)

# 3. Include Directories
# This tells the compiler to look in your new 'include' folder AND the PCL folders
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 4. Compile the executable
add_executable(buoy_lidar 
  src/main.cpp 
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
)
target_link_libraries(buoy_lidar ${PCL_LIBRARIES})

# 6. Install the Executable
install(TARGETS buoy_lidar
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
