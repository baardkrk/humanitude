#ifndef CAMERA_SUBSCRIBER_H
#define CAMERA_SUBSCRIBER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class CameraSubscriber {
 private:
  ros::NodeHandle node;
  image_transport::ImageTransport it;

  image_transport::SubscriberFilter camera_0_sub;
  image_transport::SubscriberFilter camera_0_depth_sub;

  image_transport::SubscriberFilter camera_1_sub;
  image_transport::SubscriberFilter camera_1_depth_sub;


  // cv::Mat frame_camera_0, frame_camera_1;
  cv_bridge::CvImagePtr cv_img_ptr_0;
  sensor_msgs::Image depth_ptr_0;
  cv_bridge::CvImagePtr cv_img_ptr_1;
  sensor_msgs::Image depth_ptr_1;
  
  typedef  message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::Image
    >  MySyncPolicy;

  message_filters::Synchronizer< MySyncPolicy > sync;

 public:
  CameraSubscriber();
  void callback(const sensor_msgs::ImageConstPtr& camera_0_msg,
		const sensor_msgs::ImageConstPtr& camera_1_msg,
		const sensor_msgs::Image& camera_0_depth_msg,
		const sensor_msgs::Image& camera_1_depth_msg);
  cv_bridge::CvImagePtr& get_cv_image_ptr(int i);
  sensor_msgs::Image& get_depth_ptr(int i);
  void show_images();

};

#endif // CAMERA_SUBSCRIBER_H
