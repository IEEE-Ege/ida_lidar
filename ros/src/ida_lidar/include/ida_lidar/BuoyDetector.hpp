#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

class BuoyDetector {
public:
    BuoyDetector();
    std::vector<pcl::PointIndices> processCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

private:
    void applyROI(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    bool validArcSize(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                      const pcl::PointIndices & cluster) const;

    float voxel_size_ = 0.05f;

    // Expected physical width of a buoy at any range (metres)
    static constexpr float MIN_BUOY_ARC_M = 0.1f;  // smaller = noise / glint
    static constexpr float MAX_BUOY_ARC_M = 1.5f;  // larger  = dock edge / structure
};
