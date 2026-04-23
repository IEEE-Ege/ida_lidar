cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(pcl_first_project)

# Find the PCL package and its components
find_package(PCL 1.2 REQUIRED)

# Include PCL headers and link directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Compile the executable
add_executable(voxel_filter main.cpp BuoyDetector.cpp)
# Link the PCL libraries to your executable
target_link_libraries(voxel_filter ${PCL_LIBRARIES})
