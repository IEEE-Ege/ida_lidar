#include <iostream>
#include "BuoyDetector.hpp"
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

int main(int argc, char** argv) {
    // 1. Initialize your detector
    BuoyDetector detector;

    // 2. Set up a dummy cloud (or connect to your actual LiDAR stream)
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    // ... (Load your .pcd file or get live data here) ...

    // 3. Run the pipeline
    std::vector<pcl::PointIndices> detected_buoys = detector.processCloud(raw_cloud);

    std::cout << "Found " << detected_buoys.size() << " buoys in the water!" << std::endl;

    return 0;
}