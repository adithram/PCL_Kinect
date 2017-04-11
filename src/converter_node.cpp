#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CameraInfo.h>

#include "depth_conversions.h"

namespace enc = sensor_msgs::image_encodings;
typedef sensor_msgs::PointCloud2 PointCloud;

class Converter{
public:

  Converter(ros::NodeHandle& nh) : nh_(nh){
    // initialize image transport
    image_transport::ImageTransport it(nh_);
    it_ = &it;

    // Publishers
    pub_point_cloud_ = nh_.advertise<PointCloud>("points", 1);

    // Subscribers
    cam_info_sub_ = nh_.subscribe("/camera/depth_registered/camera_info", 1, &Converter::infoCallback, this);
    depth_img_sub_ = it_->subscribe("/camera/depth_registered/image_raw", 1, &Converter::imageCallback, this);
  }

  // Callback to store info msg
  void infoCallback(const sensor_msgs::CameraInfo& info_msg){
    info_ = info_msg;
  }

  // Callback to publish point cloud
  void imageCallback(const sensor_msgs::ImageConstPtr& depth_msg){
    PointCloud::Ptr cloud_msg(new PointCloud);
    cloud_msg->header = depth_msg->header;
    cloud_msg->height = depth_msg->height;
    cloud_msg->width  = depth_msg->width;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    // Update camera model
    model_.fromCameraInfo(info_);

    if (depth_msg->encoding == enc::TYPE_16UC1){
      depth_image_proc::convert<uint16_t>(depth_msg, cloud_msg, model_);
    }

    else if (depth_msg->encoding == enc::TYPE_32FC1){
      depth_image_proc::convert<float>(depth_msg, cloud_msg, model_);
    }

    else{
      ROS_INFO("Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
      return;
    }

    pub_point_cloud_.publish (cloud_msg);

  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_point_cloud_;
  ros::Subscriber cam_info_sub_;

  image_geometry::PinholeCameraModel model_;

  sensor_msgs::CameraInfo info_;

  image_transport::ImageTransport* it_;
  image_transport::Subscriber depth_img_sub_;

}; // class Converter

int main(int argc, char **argv){
  ros::init(argc, argv, "pc_pub");

  ros::NodeHandle nh;

  Converter pc_converter(nh);

  ros::spin();
}