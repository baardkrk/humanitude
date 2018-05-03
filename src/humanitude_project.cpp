#include "person.h"
#include "pose_estimator.h"
#include <ros/ros.h>
#include <message_filters/subscriber.h>

#include "person.cpp"
#include "pose_estimator.cpp"
#include "camera_subscriber.cpp"
#include "camera_single_subscriber.cpp"

void publish_testperson() {
  geometry_msgs::Point coordinates[18];

  coordinates[0] = assign_point(0.0, .3, .2);
  coordinates[1] = assign_point(0.001, 0.001, 0.001);
  
  coordinates[2] = assign_point(-.4, .1, -.1);
  coordinates[3] = assign_point(-.6, -.2, -.3);
  coordinates[4] = assign_point(-.6, -.5, .1);

  coordinates[5] = assign_point(.4, .1, -.1);
  coordinates[6] = assign_point(.6, -.2, .1);
  coordinates[7] = assign_point(.6, -.3, .3);
 
  coordinates[8] = assign_point(-.2, -.6, .1);
  coordinates[9] = assign_point(-.3, -.6, .3);
  coordinates[10] = assign_point(-.3, -1.0, .1);
  
  coordinates[11] = assign_point(.2, -.6, 0.0);
  coordinates[12] = assign_point(.3, -.9, .1);
  coordinates[13] = assign_point(.3, -1.4, 0.0);
  
  coordinates[14] = assign_point(-.1, .4, .1);
  coordinates[15] = assign_point(.1, .4, .1);
  coordinates[16] = assign_point(-.1, .3, .0);
  coordinates[17] = assign_point(.1, .3, .0);

  Person p(478, "camera_0_link", coordinates);
  p.draw_skeleton();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "humanitude");
  ros::NodeHandle node;

  
  CameraSubscriber cam_sub;
  PoseEstimator pe;
  // CameraSingleSub css("/camera_0/qhd/image_color_rect");
  // Person p; //p.draw_skeleton();
  while(node.ok()) {
    // p.draw_skeleton();
    //    css.show_image();
    // cam_sub.show_images();
    // publish_testperson();
    if (cam_sub.get_cv_image_ptr(0) != nullptr)
      pe.estimate_poses(cam_sub.get_cv_image_ptr(0)->image, cam_sub.get_depth_ptr(0));
    ros::spinOnce();
  }
  
  return 0;
}
