#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <dynamic_reconfigure/server.h>
#include <rover_navigation/ObstacleAvoidConfig.h>

#include "segmentation.h"
#include "obstacle_depth_node.h"
#include "Obstacle.h"
#include "rover_navigation/ObstaclesMsg.h"

namespace enc = sensor_msgs::image_encodings;

namespace obstacle_detect{


ObstacleDepthNode::ObstacleDepthNode() : bg_img_count_(0), 
                                                                         is_obs_avoid_mode_(false), 
                                                                         is_bg_img_init_(false) {}

void ObstacleDepthNode::bgMode(cv::Mat& depth_img){
    ROS_INFO("BG Mode callback activated");
    if (! is_bg_img_init_){
        bg_img_ = depth_img.clone();
        bg_img_count_ = 1;
        is_bg_img_init_ = true;

        // Convert to floats so it can be summed
        bg_img_.convertTo(bg_img_, CV_32FC1);

        ROS_INFO("Starting obstacle avoidance");

        return;
    }

    cv::add(bg_img_, depth_img, bg_img_);
    ++bg_img_count_;
    ROS_INFO("Just added a new background image");

    return;
}

void ObstacleDepthNode::obsDetect(cv::Mat& depth_img){
    // TODO: Make it a struct object -- x,y, width 
    std::vector<Obstacle> obstacles;
    ROS_INFO("Obstacle Detect callback activated");
    cv::absdiff(depth_img, bg_img_, depth_img);
    ROS_INFO("abd diff calculated");

    segmentation::segmentDepthImage(depth_img, bg_img_, obstacles);

    int obstacle_vector_size = (int)obstacles.size();

    ROS_INFO("Found %i obstacles", obstacle_vector_size);

    ros::NodeHandle nh;
    
    rover_navigation::SquareObstacle square_obstacle;
    rover_navigation::ObstaclesMsg message_object_to_send;


    for (int i = 0; i < obstacle_vector_size; i++){
        square_obstacle.x = obstacles[i].x;
        square_obstacle.y = obstacles[i].y;
        square_obstacle.average_z = obstacles[i].average_z;
        square_obstacle.width = obstacles[i].width;
        square_obstacle.height = obstacles[i].height;
        message_object_to_send.squares.push_back(square_obstacle);

    }

    message_object_to_send.size = obstacle_vector_size;

    ros::Publisher obstacle_pub = nh.advertise<rover_navigation::ObstaclesMsg>("Obstacles", 1);
    obstacle_pub.publish(message_object_to_send);

    // 1 Hz update
    ros::Duration(1).sleep();
    return;
}

void ObstacleDepthNode::dynReconfigureCb(rover_navigation::ObstacleAvoidConfig &config, uint32_t level) {
    if (config.is_obs_avoid_mode_){
        // Signal came in
        is_obs_avoid_mode_ = true;

        // Take the average of the background image, convert back to 8UC
        bg_img_ = bg_img_ / bg_img_count_;
        bg_img_.convertTo(bg_img_, CV_8UC1);
        ROS_INFO("Reaveraged background image");
      
        // snaps the dynamic reconfigure parameter back to false
        // makes testing a bit easier / less frustrating
        config.is_obs_avoid_mode_ = false;
    }

    else{
        is_obs_avoid_mode_ = false;
    }

    return;
}

void ObstacleDepthNode::depthImageCb(const sensor_msgs::ImageConstPtr& msg){
    // // TODO: Subtract the image from the background image -- make this into a class
    ROS_INFO("Inside depthImageCb!");

    cv_bridge::CvImagePtr depth_img;
    try{
        if (is_obs_avoid_mode_){
            depth_img = cv_bridge::toCvCopy(msg , sensor_msgs::image_encodings::TYPE_8UC1);
        }
        else{
            depth_img = cv_bridge::toCvCopy(msg , sensor_msgs::image_encodings::TYPE_32FC1);  
        }
    }

    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (is_obs_avoid_mode_){
        //Normalize the pixel value
        // TODO: See if this is actually useful in testing
        ROS_INFO("Normalizing...");
        cv::normalize(depth_img->image, depth_img->image, 0, 255, cv::NORM_MINMAX, CV_8UC1 );
        obsDetect(depth_img->image);
    }

    else bgMode(depth_img->image);
}


void ObstacleDepthNode::run(){
    // Dynamic Reconfigure setup
    dynamic_reconfigure::Server<rover_navigation::ObstacleAvoidConfig> server;
    dynamic_reconfigure::Server<rover_navigation::ObstacleAvoidConfig>::CallbackType f;

    f = boost::bind(&ObstacleDepthNode::dynReconfigureCb, this, _1, _2);
    server.setCallback(f);

    // Node setup
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub;

    image_sub = it.subscribe("/camera/depth/image_raw", 1, &ObstacleDepthNode::depthImageCb, this);

    ROS_INFO("Fully initialized obstacle_depth_node, starting");
    
    ros::spin();
}

}; // namespace obstacle_detect

int main(int argc, char** argv){
    ros::init(argc, argv, "obstacle_depth_node");
    
    obstacle_detect::ObstacleDepthNode obavoid_instance;
    obavoid_instance.run();
}
