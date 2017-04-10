#include <ros/ros.h>

#include "pc_background_subtract.h"

using namespace BackgroundSubtract;

 /*

 // TODO:
 // Grabber - Use depth_image_proc from ROS and use a depth image to point cloud conversion
 // Kinect Error Modelling
  
 */
int main(int argc, char** argv) {
    
    ros::init(argc, argv, "obstacle_pc_node");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZ> base_cloud;

    initializeBaseCloud(base_cloud);
      
    
    while(ros::ok()){
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
        
        ros::spinOnce();
    
    }
    
    return (0);
}


