#include <stdio.h>
#include <errno.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "withrobot_camera.hpp" /* withrobot camera API */

class oCam_ROS
{
public:
  oCam_ROS();
  ~oCam_ROS();

private:
  Withrobot::Camera* camera_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  image_transport::ImageTransport it_;
  image_transport::CameraPublisher image_pub_;
  image_transport::Publisher mono_image_pub_;

  sensor_msgs::CameraInfo info_;
  camera_info_manager::CameraInfoManager info_manager_;

  int width_;
  int height_;
  int fps_;
  std::string camera_info_url_;
  std::string device_path_;
  std::string image_topic_;
  std::string mono_image_topic_;
  std::string frame_id_;
  bool show_image_;
  bool rescale_camera_info_;
  bool auto_exposure_;

  void rescaleCameraInfo(int width, int height);
};
