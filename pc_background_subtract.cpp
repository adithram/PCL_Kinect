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
    cout << "Saved " << base_cloud.points.size () << " data points to base_pcd.pcd." << endl;

    // Write base_cloud to terminal
    cout << "x" << " " << "y" << " " << "z" << endl; // I know this could be one line, but this is easier to read for me. 
    for (size_t i = 0; i < base_cloud.points.size (); ++i){
      cout << " " << base_cloud.points[i].x << " " << base_cloud.points[i].y << " " << base_cloud.points[i].z << endl;
    }
  }

  // function to initialize random comparison cloud
  void initializeCompareCloud(pcl::PointCloud<pcl::PointXYZ> &base_cloud, pcl::PointCloud<pcl::PointXYZ> &compare_cloud){

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
    cout << "Saved " << base_cloud.points.size () << " data points to compare_cloud.pcd." << endl;
  }

  // function that sets z of compare cloud to the abs difference of the two point cloud values
  void compareClouds(pcl::PointCloud<pcl::PointXYZ> &base_cloud, pcl::PointCloud<pcl::PointXYZ> &compare_cloud){
      
    for (size_t i = 0; i < base_cloud.size(); ++i){
        compare_cloud.points[i].z = fabs(base_cloud.points[i].z - compare_cloud.points[i].z);
    }
  }

  // function to smooth cloud to fill in depth uncertainties
  void smoothCloud(pcl::PointCloud<pcl::PointXYZ> &compare_cloud){
    return;
  }

  // function to group, extract clusters from an input point cloud
  // Returns vector of pointers to pointclouds, each representing a cluster
  vector< pcl::PointCloud<pcl::PointXYZ> * > extractClustersFromCloud(pcl::PointCloud<pcl::PointXYZ> &compare_cloud){

    //Initialize kdTree and feed compare cloud into the tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (compare_cloud);

    // Vector that contains the cluster indices
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    //How do we test these values?
    /*
    Be careful setting the right value for setClusterTolerance(). If you take a very small value, 
    it can happen that an actual object can be seen as multiple clusters. On the other hand, if you set the 
    value too high, it could happen, that multiple objects are seen as one cluster. So our recommendation is to 
    just test and try out which value suits your dataset.
    */
    ec.setClusterTolerance (0.02); // 2cm
    // We impose that the clusters found must have at least setMinClusterSize() points and maximum setMaxClusterSize() points.
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (compare_cloud);
    ec.extract (cluster_indices);

    int j = 0;
    vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ()
    for (it; it != cluster_indices.end (); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

      vector<int>::const_iterator pit = it->indices.begin ()
      for (pit; pit != it->indices.end (); ++pit) {
        cloud_cluster->points.push_back (compare_cloud->points[*pit]); //*
      }
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      //Vector of pointers to point clouds. 
      vector< pcl::PointCloud<pcl::PointXYZ> * > cluster_clouds;

      // Writing values to a PCD file
      // Return vector of point clouds (each representing cluster) ?
      cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
      stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
      j++;

      //Push cloud into vector
      cluster_clouds.push_back(cloud_cluster);
    } //Closing for loop iterating through cluster_indices

    return cluster_clouds;

  } // closing extract function

  // function that averages values in pointcloud, returns vector of average location (x,y, length and width)
  // Should return average x, y, length, and width (approximate) of obstacles (the cluster)
  // values contained in obstacle object
  vector<Obstacle> extractAverages(vector< pcl::PointCloud<pcl::PointXYZ> * > cluster_clouds){
    vector<Obstacle> obstacles_vector;
    return obstacles_vector;
  }


} // Closing namespace