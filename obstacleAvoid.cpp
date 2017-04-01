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

        // extract clusters: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
        // return in a vector format?
        extractClustersFromCloud(compare_cloud);


        // Slow down data rate
        this_thread::sleep_for(chrono::seconds(2)); 
        
    
    }
    
    return (0);
  }

} // namespace BackgroundSubtract

