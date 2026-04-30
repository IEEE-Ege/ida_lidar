#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

class BuoyDetector {
public:
    // Constructor
    BuoyDetector();

    // The main public function you will call from outside
    std::vector<pcl::PointIndices> processCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

private:
    // Internal helper functions for each step of the pipeline
    void applyROI(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void removeWaterSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Internal variables (e.g., thresholds you might want to tune)
    float voxel_size_ = 0.1f;
    float water_distance_threshold_ = 0.2f;
};