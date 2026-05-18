#include "ida_lidar/BuoyDetector.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <cmath>
#include <cfloat>

// 2D ROI — sized to RPLIDAR C1's reliable outdoor range
static constexpr float ROI_X_MIN = -10.0f, ROI_X_MAX = 10.0f;
static constexpr float ROI_Y_MIN = -10.0f, ROI_Y_MAX = 10.0f;

BuoyDetector::BuoyDetector() {}

std::vector<pcl::PointIndices> BuoyDetector::processCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
    applyROI(input_cloud);
    downsample(input_cloud);

    if (input_cloud->empty()) { return {}; }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> raw_clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.3);  // 2D points on a buoy arc are close together
    ec.setMinClusterSize(3);      // a buoy at max range returns ~4 pts — 3 is the floor
    ec.setMaxClusterSize(50);     // arc segments are small; anything larger is structure
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(raw_clusters);

    // Reject clusters whose physical width doesn't match a buoy
    std::vector<pcl::PointIndices> valid_clusters;
    for (const auto & cluster : raw_clusters) {
        if (validArcSize(input_cloud, cluster)) {
            valid_clusters.push_back(cluster);
        }
    }

    return valid_clusters;
}

void BuoyDetector::applyROI(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);

    // 2D sensor — only X and Y are meaningful, no Z filter
    pass.setFilterFieldName("x");
    pass.setFilterLimits(ROI_X_MIN, ROI_X_MAX);
    pass.filter(*cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(ROI_Y_MIN, ROI_Y_MAX);
    pass.filter(*cloud);
}

void BuoyDetector::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    vg.filter(*cloud);
}

bool BuoyDetector::validArcSize(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                                 const pcl::PointIndices & cluster) const
{
    float min_angle =  FLT_MAX;
    float max_angle = -FLT_MAX;
    float dist_sum  =  0.0f;

    for (int idx : cluster.indices) {
        const auto & pt = cloud->points[idx];
        float angle = std::atan2(pt.y, pt.x);
        float dist  = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        if (angle < min_angle) min_angle = angle;
        if (angle > max_angle) max_angle = angle;
        dist_sum += dist;
    }

    // Chord length = average_distance × angular_span (small angle approximation)
    float avg_dist   = dist_sum / static_cast<float>(cluster.indices.size());
    float chord      = avg_dist * (max_angle - min_angle);

    return chord >= MIN_BUOY_ARC_M && chord <= MAX_BUOY_ARC_M;
}
