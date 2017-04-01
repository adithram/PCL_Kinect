#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <chrono>
#include <thread>
#include <tuple>
#include <stdio.h>   
#include <stdlib.h> 
#include <cmath>

#include "pc_background_subtract.h"
#include <"obstacle.h">


using namespace std;

 /*

 // TODO:
 // Grabber
 // Kinect Error Modelling
 // Extract clusters
  
 */
namespace BackgroundSubtract{ 

  int main (int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZ> base_cloud;

    initiailzeBaseCloud(base_cloud);
      
    
    while(1){
        pcl::PointCloud<pcl::PointXYZ> compare_cloud; 

        initializeCompareCloud(base_cloud, compare_cloud);

        // compare_cloud now has "differences"
        compareClouds(base_cloud, compare_cloud);
        
        // Smoothing kernel on point cloud
        smoothCloud(compare_cloud);

        // Extract clusters ; return vector containing cluster locations
        vector< pcl::PointCloud<pcl::PointXYZ> * > cluster_clouds;
        cluster_clouds = extractClustersFromCloud(compare_cloud);

        vector<Obstacle> obstacles_vector;
        obstacles_vector = extractAverages(cluster_clouds);

        //publish the vector

        // Slow down data rate
        this_thread::sleep_for(chrono::seconds(2)); 
        
    
    }
    
    return (0);
  }

} // namespace BackgroundSubtract

