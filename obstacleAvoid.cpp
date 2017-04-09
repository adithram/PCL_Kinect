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

    initializeBaseCloud(base_cloud);
      
    
    while(1){
        pcl::PointCloud<pcl::PointXYZ> compare_cloud; 

        initializeCompareCloud(base_cloud, compare_cloud);

        // compare_cloud now has "differences"
        compareClouds(base_cloud, compare_cloud);
        
        // Smoothing kernel on point cloud
        smoothCloud(compare_cloud);

        // Extract clusters ; return vector containing cluster locations
        vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_clouds;
        cluster_clouds = extractClustersFromCloud(compare_cloud);

        vector<Obstacle> obstacles_vector;
        obstacles_vector = extractAverages(cluster_clouds);

        //publish the vector
        //temporarily write into the terminal
        terminalWrite(obstacles_vector);

        // Slow down data rate
        this_thread::sleep_for(chrono::seconds(2)); 
        
    
    }
    
    return (0);
  }

} // namespace BackgroundSubtract

