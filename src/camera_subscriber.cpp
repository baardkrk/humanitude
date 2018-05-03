#include "camera_subscriber.h"

CameraSubscriber::CameraSubscriber() :
  it(node),
  camera_0_sub(it, "/camera_0/qhd/image_color_rect", 1),
  camera_1_sub(it, "/camera_1/qhd/image_color_rect", 1),
  camera_0_depth_sub(it, "/camera_0/qhd/image_depth_rect", 1),
  camera_1_depth_sub(it, "/camera_1/qhd/image_depth_rect", 1),
  sync(MySyncPolicy(100), camera_0_sub, camera_1_sub, camera_0_depth_sub, camera_1_depth_sub)
{
  sync.registerCallback(boost::bind(&CameraSubscriber::callback, this, _1, _2, _3, _4));
  cv_img_ptr_0 = cv_img_ptr_1 = nullptr;
  //    *depth_ptr_0 = *depth_ptr_1 = nullptr;
};

void CameraSubscriber::callback(const sensor_msgs::ImageConstPtr& camera_0_msg,
				const sensor_msgs::ImageConstPtr& camera_1_msg,
				const sensor_msgs::Image& camera_0_depth_msg,
				const sensor_msgs::Image& camera_1_depth_msg) {

  //  cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(camera_0_msg);
  try {
    cv_img_ptr_0 = cv_bridge::toCvCopy(camera_0_msg);
    cv_img_ptr_1 = cv_bridge::toCvCopy(camera_1_msg);
    depth_ptr_0 = camera_0_depth_msg;
    depth_ptr_1 = camera_1_depth_msg;

    // ROS_INFO("registered callback");
  } catch (cv_bridge::Exception& e) {ROS_ERROR("cv_bridge exception: %s\n", e.what()); return;}
};

cv_bridge::CvImagePtr& CameraSubscriber::get_cv_image_ptr(int i) {
  return (i == 0) ? cv_img_ptr_0 : cv_img_ptr_1;
};

sensor_msgs::Image& CameraSubscriber::get_depth_ptr(int i) {
  return (i == 0) ? depth_ptr_0 : depth_ptr_1;
}

void CameraSubscriber::show_images() {
  cv::Mat tmp;//, res;

  if (cv_img_ptr_0 != nullptr && cv_img_ptr_1 != nullptr) {
    
    
    cv::hconcat(cv_img_ptr_0->image, cv_img_ptr_1->image, tmp);
    // cv::hconcat(cv_depth_ptr_0->image, cv_depth_ptr_1->image, res);
    // cv::vconcat(tmp, res, res);
    
    cv::imshow("color images", tmp);
    // cv::imshow("depth images", res);
    cv::waitKey(1);
  }
  // if (cv_img_ptr_0 != nullptr) {
  //   cv::imshow("Preview cam 0", cv_img_ptr_0->image);
  //   cv::waitKey(1);
  // }
  // if (cv_depth_ptr_0 != nullptr) {
  //   cv::imshow("Depth cam 0", cv_depth_ptr_0->image);
  //   cv::waitKey(1);
  // }
  // ROS_INFO("did not aquire image..");
};

