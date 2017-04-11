#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "segmentation.h"

bool GLOBAL_BOOL = false;

void depthImageCb(const sensor_msgs::ImageConstPtr& msg){
    if (GLOBAL_BOOL) return;
    // TODO: Subtract the image from the background image -- make this into a class
    ROS_INFO("Inside depthImageCb!");

    cv_bridge::CvImageConstPtr cv_ptr;

    try{
        cv_ptr =  cv_bridge::toCvShare(msg);
    }

    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // TODO: Make it a struct object -- x,y, width 
    std::vector<std::pair<int, int>> obstacles;

    Segmentation::segmentDepthImage(cv_ptr->image, obstacles);
    GLOBAL_BOOL = true;
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "obstacle_depth_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub;

    // TODO: Fix path
    image_sub = it.subscribe("/camera/depth_registered/image_raw", 1, &depthImageCb);

    ros::spin();

}