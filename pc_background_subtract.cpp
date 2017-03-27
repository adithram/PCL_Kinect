#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <chrono>
#include <thread>
#include <tuple>
#include <stdio.h>   
#include <stdlib.h> 
#include <cmath>

using namespace std;

 /*

 // TODO:
 // Grabber
 // Kinect Error Modelling
 // Extract clusters
  
 */

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
  cout << "x" << " " << "y" << " " << "z" << endl; // I know this could be one line, but this is easier to read for me. ("aka i have autism" - bhiarva)
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

/* 
// No need for this code but keeping for reference

//Perform background subtraction between base_cloud and compare_cloud.
  
// Vector that contains the differences between specific points.
vector<boost::tuple<double, double, double> > dif_points;

for (size_t i = 0; i < base_cloud.size(); ++i){
      double z = fabs(base_cloud.points[i].z - compare_cloud.points[i].z);
  
      //Construct tuple
      double x = base_cloud.points[i].x;
      double y = base_cloud.points[i].y;
      boost::tuple<double, double, double> point (x, y, z);
    
      //Add to vector
      dif_points.push_back(point);
}

// vector that contains the points that serve as locations for obstacles.
vector<pair<double, double> > obstacle_locations; 

for ( unsigned int i = 0; i < dif_points.size(); ++i){
  // Define threshold for z value
  if( get<2>(dif_points[i]) > .5){
    double x = get<0>(dif_points[i]);
    double y = get<1>(dif_points[i]);
    pair<double, double> point (x,y);
    obstacle_locations.push_back(point);
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------
// TODO: EXTRACT CLUSTERS
//--------------------------------------------------------------------------------------------------------------------------------------------

// Publish the points when integrating to ROS. For now, printint to stdout.
for (unsigned int i = 0; i < obstacle_locations.size(); ++i){
  cout << "Obstacle at: (" << obstacle_locations[i].first << ", " << obstacle_locations[i].second << ")" << endl;
}

*/ 