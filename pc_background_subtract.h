#ifndef PC_BACKGROUND_SUBTRACT_H
#define PC_BACKGROUND_SUBTRACT_H

#include <pcl/point_types.h>
#include <"obstacle.h">

namespace BackgroundSubtract {
    
// Function to randomly initialize base pointcloud
void initiailzeBaseCloud(pcl::PointCloud<pcl::PointXYZ> &base_cloud);

// function to initialize random comparison cloud
void initializeCompareCloud(pcl::PointCloud<pcl::PointXYZ> &base_cloud, 
    pcl::PointCloud<pcl::PointXYZ> &compare_cloud);

void compareClouds(pcl::PointCloud<pcl::PointXYZ> &base_cloud, 
    pcl::PointCloud<pcl::PointXYZ> &compare_cloud);

void smoothCloud(pcl::PointCloud<pcl::PointXYZ> &compare_cloud);

vector< pcl::PointCloud<pcl::PointXYZ> * > extractClustersFromCloud(pcl::PointCloud<pcl::PointXYZ> &compare_cloud);

// function that averages values in pointcloud, returns vector of average location (x,y, length and width)
// Should return average x, y, length, and width (approximate) of obstacles (the cluster)
// values contained in obstacle object
vector<Obstacle> extractAverages(vector< pcl::PointCloud<pcl::PointXYZ> * > cluster_clouds);

} // namespace BackgroundSubtract

#endif // PC_BACKGROUND_SUBTRACT_H