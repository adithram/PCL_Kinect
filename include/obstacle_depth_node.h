#ifndef OBSTACLE_DEPTH_NODE_H
#define OBSTACLE_DEPTH_NODE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>

namespace obstacle_detect{

class ObstacleDepthNode{

public:
    ObstacleDepthNode();
    void bgMode(cv::Mat& depth_img);
    void obsDetect(cv::Mat& depth_img);
    void dynReconfigureCb(rover_navigation::ObstacleAvoidConfig &config, uint32_t level);
    void depthImageCb(const sensor_msgs::ImageConstPtr& msg);
    void run();


private:

    cv::Mat bg_img_;

    int bg_img_count_;

    bool is_obs_avoid_mode_;
    bool is_bg_img_init_;

}; // ObstacleDepthNode


}; // namespace obstacle_detect

#endif // OBSTACLE_DEPTH_NODE_H