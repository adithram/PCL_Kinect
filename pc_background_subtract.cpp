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

namespace BackgroundSubtract{ 

// Function to randomly initialize base pointcloud
  void initiailzeBaseCloud(pcl::PointCloud<pcl::PointXYZ> &base_cloud){
    // Faking point cloud temporarily. 
    // Fill in the cloud data
    base_cloud.width = 5;
    base_cloud.height = 1;
    base_cloud.is_dense = false;
    base_cloud.points.resize (base_cloud.width * base_cloud.height);
    
    int cols = 1;
    
    for (size_t i = 0; i < base_cloud.points.size (); ++i)
    {
      base_cloud.points[i].x = i % cols;
      base_cloud.points[i].y = i / cols;
      base_cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
      cols++;
    }
    
    cols = 1;

    // Write base_cloud to pcd file for testing
    pcl::io::savePCDFileASCII ("base_pcd.pcd", base_cloud);
    cout << "Saved " << base_cloud.points.size () << " data points to base_pcd.pcd." << std::endl;

    // Write base_cloud to terminal
    cout << "x" << " " << "y" << " " << "z" << endl; // I know this could be one line, but this is easier to read for me. 
    for (size_t i = 0; i < base_cloud.points.size (); ++i){
      cout << " " << base_cloud.points[i].x << " " << base_cloud.points[i].y << " " << base_cloud.points[i].z << endl;
    }
  }

  // function to initialize random comparison cloud
  void initializeCompareCloud(pcl::PointCloud<pcl::PointXYZ> &base_cloud, 
      pcl::PointCloud<pcl::PointXYZ> &compare_cloud){

    compare_cloud.width = 5;
    compare_cloud.height = 1;
    compare_cloud.is_dense = false;
    compare_cloud.points.resize (compare_cloud.width * compare_cloud.height);
          
    for (size_t i = 0; i < compare_cloud.size(); ++i){
      compare_cloud.points[i].x = i % cols; 
      compare_cloud.points[i].y = i / cols;
      compare_cloud.points[i].z = base_cloud.points[i].z + rand() % 3 + (-1); // generates a random fluctuation of +/- 1 or 0
      cols++;
    }

    // Write base_cloud to pcd file for testing
    pcl::io::savePCDFileASCII ("compare_cloud.pcd", compare_cloud);
    cout << "Saved " << base_cloud.points.size () << " data points to compare_cloud.pcd." << std::endl;
  }

  // function that sets z of compare cloud to the abs difference of the two point cloud values
  void compareClouds(pcl::PointCloud<pcl::PointXYZ> &base_cloud, 
      pcl::PointCloud<pcl::PointXYZ> &compare_cloud){
      
    for (size_t i = 0; i < base_cloud.size(); ++i){
        compare_cloud.points[i].z = fabs(base_cloud.points[i].z - compare_cloud.points[i].z);
    }
  }

  // function to smooth cloud to fill in depth uncertainties
  void smoothCloud(pcl::PointCloud<pcl::PointXYZ> &compare_cloud){
    return;
  }

  // function to group, extract clusters from an input point cloud
  // Should return average x, y, length, and width (approximate) of obstacles (the cluster)
  void extractClustersFromCloud(pcl::PointCloud<pcl::PointXYZ> &compare_cloud){
    return;
  }

}