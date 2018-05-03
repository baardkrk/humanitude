#include "camera_single_subscriber.h"


CameraSingleSub::CameraSingleSub(const std::string& image_topic) :
  it(node) {
  image_sub = it.subscribe(image_topic, 1, &CameraSingleSub::callback, this);
  cv_img_ptr = nullptr;
};

void CameraSingleSub::callback(const sensor_msgs::ImageConstPtr& msg) {
  try{
    // Omitting encoding, since it leads to problems with '16UC1' encoding
    cv_img_ptr = cv_bridge::toCvCopy(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s\n", e.what());
  }
};

cv_bridge::CvImagePtr& CameraSingleSub::getCvImagePtr() {
  return cv_img_ptr;
};

void CameraSingleSub::show_image() {
  if (cv_img_ptr != nullptr)
    cv::imshow("preview", cv_img_ptr->image);
};
