#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include <cv_bridge/cv_bridge.h>
#include <string>

///////// OpenPose dependencies //////////
#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>


#include "openpose_flags.cpp"
// #include "person.cpp"

class PoseEstimator {
 private:
  std::string cam_0;
  std::string cam_1;

  op::Array<float> pose_keypoints_0;
  op::Array<float> pose_keypoints_1;

  std::vector<Person> people_0;
  std::vector<Person> people_1;
  
 public:
  PoseEstimator();
  void run_openpose(cv::Mat frame, op::Array<float> *pose_keypoints);
  void estimate_poses(cv::Mat frame, sensor_msgs::Image depth_frame);
};


#endif // POSE_ESTIMATOR_H
