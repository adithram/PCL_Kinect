#ifndef PC_BACKGROUND_SUBTRACT_H
#define PC_BACKGROUND_SUBTRACT_H

#include <pcl/point_types.h>

namespace BackgroundSubtract {
    
// Function to randomly initialize base pointcloud
void initiailzeBaseCloud(pcl::PointCloud<pcl::PointXYZ> &base_cloud);

// function to initialize random comparison cloud
void initializeCompareCloud(pcl::PointCloud<pcl::PointXYZ> &base_cloud, 
    pcl::PointCloud<pcl::PointXYZ> &compare_cloud);

void compareClouds(pcl::PointCloud<pcl::PointXYZ> &base_cloud, 
    pcl::PointCloud<pcl::PointXYZ> &compare_cloud);

void smoothCloud(pcl::PointCloud<pcl::PointXYZ> &compare_cloud);

void extractClustersFromCloud(pcl::PointCloud<pcl::PointXYZ> &compare_cloud);

} // namespace BackgroundSubtract

#endif // PC_BACKGROUND_SUBTRACT_H