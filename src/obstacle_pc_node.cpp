#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "pc_background_subtract.h"

using namespace BackgroundSubtract;

 /*

 // TODO:
 // Grabber
 // Kinect Error Modelling
 // Extract clusters
  
 */

//Adding point cloud class so that base_cloud is a member variable
class PointCloudProcessing {
public:
    void pointCloudCb(const sensor_msgs::PointCloud2& pc_msg);
    pcl::PointCloud<pcl::PointXYZ> base_cloud;

};

void PointCloudProcessing::pointCloudCb(const sensor_msgs::PointCloud2& pc_msg){
    ROS_INFO("Inside PC Callback!");

    
    pcl::PointCloud<pcl::PointXYZ> compare_cloud; 

    initializeCompareCloud(base_cloud, compare_cloud);

    // compare_cloud now has "differences"
    compareClouds(base_cloud, compare_cloud);
    
    // Smoothing kernel on point cloud
    smoothCloud(compare_cloud);

    // Extract clusters ; return vector containing cluster locations
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_clouds;
    cluster_clouds = extractClustersFromCloud(compare_cloud);

    std::vector<Obstacle> obstacles_vector;
    obstacles_vector = extractAverages(cluster_clouds);

    //publish the vector
    //temporarily write into the terminal
    terminalWrite(obstacles_vector);

    ros::Duration(0.5).sleep();
    
}


int main(int argc, char** argv) {


    ros::init(argc, argv, "obstacle_pc_node");
    ros::NodeHandle nh;
 
    // instantiate class
    PointCloudProcessing pcp_instance;

    // pcl::PointCloud<pcl::PointXYZ> base_cloud;

    initializeBaseCloud(pcp_instance.base_cloud);

    ros::Subscriber sub;
    sub = nh.subscribe("points", 1, &PointCloudProcessing::pointCloudCb, &pcp_instance);
    
    ros::spin();
    
    return 0;
}


