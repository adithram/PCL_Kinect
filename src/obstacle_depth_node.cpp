#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "segmentation.h"

namespace enc = sensor_msgs::image_encodings;

bool GLOBAL_BOOL = false;


void depthImageCb(const sensor_msgs::ImageConstPtr& msg){
    if (GLOBAL_BOOL) return;

    // // Allocate new Image message
    // sensor_msgs::ImagePtr depth_msg( new sensor_msgs::Image );
    // depth_msg->header   = msg->header;
    // depth_msg->encoding = enc::TYPE_32FC1;
    // depth_msg->height   = msg->height;
    // depth_msg->width    = msg->width;
    // depth_msg->step     = msg->width * sizeof (float);
    // depth_msg->data.resize( depth_msg->height * depth_msg->step);

    // float bad_point = std::numeric_limits<float>::quiet_NaN ();

    // // Fill in the depth image data, converting mm to m
    // const uint16_t* raw_data = reinterpret_cast<const uint16_t*>(&msg->data[0]);
    // float* depth_data = reinterpret_cast<float*>(&depth_msg->data[0]);

    // for (unsigned index = 0; index < depth_msg->height * depth_msg->width; ++index){
    //     uint16_t raw = raw_data[index];
    //     depth_data[index] = (raw == 0) ? bad_point : (float)raw * 0.001f;
    // }


    // // TODO: Subtract the image from the background image -- make this into a class
    ROS_INFO("Inside depthImageCb!");
    cv_bridge::CvImagePtr depthImg;
    try{
        depthImg = cv_bridge::toCvCopy(msg , sensor_msgs::image_encodings::TYPE_8UC1);
    }

    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // TODO: Make it a struct object -- x,y, width 
    std::vector<std::pair<int, int>> obstacles;

    //Normalize the pixel value
    cv::normalize(depthImg->image, depthImg->image, 0, 255, cv::NORM_MINMAX, CV_8UC1 );

    segmentation::segmentDepthImage(depthImg->image, obstacles);
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