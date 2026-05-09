#include "ida_lidar/BuoyDetector.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

// ROI limits — tune these during water trials
static constexpr float ROI_X_MIN = -30.0f, ROI_X_MAX = 30.0f;  // left/right
static constexpr float ROI_Y_MIN = -30.0f, ROI_Y_MAX = 30.0f;  // forward/back
static constexpr float ROI_Z_MIN =  -0.5f, ROI_Z_MAX =  3.0f;  // height

BuoyDetector::BuoyDetector() {
    // Initialization code if needed
}

std::vector<pcl::PointIndices> BuoyDetector::processCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    // 1. ROI
    applyROI(input_cloud);
    
    // 2. Voxel Grid
    downsample(input_cloud);
    
    // 3. RANSAC Water Removal
    removeWaterSurface(input_cloud);
    
    // 4. Clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5);  // max distance between points in the same cluster (metres)
    ec.setMinClusterSize(5);      // ignore clusters with fewer points (noise)
    ec.setMaxClusterSize(500);    // ignore clusters with more points (not a buoy)
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(clusters);

    return clusters;
}

void BuoyDetector::applyROI(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(ROI_X_MIN, ROI_X_MAX);
    pass.filter(*cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(ROI_Y_MIN, ROI_Y_MAX);
    pass.filter(*cloud);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(ROI_Z_MIN, ROI_Z_MAX);
    pass.filter(*cloud);
}

void BuoyDetector::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    vg.filter(*cloud);
}

void BuoyDetector::removeWaterSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(water_distance_threshold_);
    seg.setInputCloud(cloud);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        return; // No dominant plane found, leave cloud untouched
    }

    // Remove the inliers (water plane), keep everything above it
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // true = keep the NON-plane points
    extract.filter(*cloud);
}
