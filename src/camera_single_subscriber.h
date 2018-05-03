#ifndef CAMERA_SINGLE_SUB_H
#define CAMERA_SINGLE_SUB_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class CameraSingleSub {
 private:
  ros::NodeHandle node;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  cv_bridge::CvImagePtr cv_img_ptr;

 public:
  CameraSingleSub(const std::string& image_topic);
  void callback(const sensor_msgs::ImageConstPtr& msg);
  cv_bridge::CvImagePtr& getCvImagePtr();
  void show_image();

};

#endif // CAMERA_SINGLE_SUB_H
