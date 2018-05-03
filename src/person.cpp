//#include "person.h"
#include <iostream>
#include <sstream>
#include <cmath>


std::string Person::bpnames[] = {"nose", "neck", "r_shoul", "r_elbow", "r_wrist", "l_shoul", "l_elbow", "l_wrist", "r_hip", "r_knee", "r_ankle", "l_hip", "l_knee", "l_ankle", "r_eye", "l_eye", "r_ear", "l_ear"};

geometry_msgs::Point assign_point(float x, float y, float z)
{
  geometry_msgs::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
};

Person::Person(int _id, std::string _base_frame, geometry_msgs::Point _coordinates[18]) {

  id = _id;
  // coordinates = _coordinates;
  std::copy(_coordinates, _coordinates+18, coordinates);
  base_frame = _base_frame;
  
  // initializing points for testing ---- TODO remove this for final version
  
  // coordinates[0] = assign_point(0.0, 3.0, 2.0);
  // coordinates[1] = assign_point(0.01, 0.01, 0.01);
  
  // coordinates[2] = assign_point(-4.0, 1.0, -1.0);
  // coordinates[3] = assign_point(-6.0, -2.0, -3.0);
  // coordinates[4] = assign_point(-6.0, -5.0, 1.0);

  // coordinates[5] = assign_point(4.0, 1.0, -1.0);
  // coordinates[6] = assign_point(6.0, -2.0, 1.0);
  // coordinates[7] = assign_point(6.0, -3.0, 3.0);
 
  // coordinates[8] = assign_point(-2.0, -6.0, 1.0);
  // coordinates[9] = assign_point(-3.0, -6.0, 3.0);
  // coordinates[10] = assign_point(-3.0, -10.0, 1.0);
  
  // coordinates[11] = assign_point(2.0, -6.0, 0.0);
  // coordinates[12] = assign_point(3.0, -9.0, 1.0);
  // coordinates[13] = assign_point(3.0, -14.0, 0.0);
  
  // coordinates[14] = assign_point(-1.0, 4.0, 1.0);
  // coordinates[15] = assign_point(1.0, 4.0, 1.0);
  // coordinates[16] = assign_point(-1.0, 3.0, 0.0);
  // coordinates[17] = assign_point(1.0, 3.0, 0.0);

  // base_frame = "camera_0_link";
  marker_pub = node.advertise<visualization_msgs::Marker>("skeleton_markers", 100);
};

void Person::broadcast_keypoint_frames() {
  
};

void hsv_to_rgb(int h, int s, int v, int *r, int *g, int *b) {
  
  float rt, gt, bt;
  // ensure correct values for hsv
  h = abs(h); s = abs(s); v = abs(v);
  if (h > 359) h = h % 359;
  if (s > 1) s = abs(s / (floor(std::log10(s)) * 10));
  if (v > 1) v = abs(v / (floor(std::log10(v)) * 10));
  
  float C = (float)(s * v); // should be 1 anyway
  float X = (float)(C * (1.0 - std::abs(fmod((h/60.0),2) -1.0)));
  float m = (float)(v - C);
  
  if      (h <  60) { rt = C; gt = X; bt = 0; }
  else if (h < 120) { rt = X; gt = C; bt = 0; }
  else if (h < 180) { rt = 0; gt = C; bt = X; }
  else if (h < 240) { rt = 0; gt = X; bt = C; }
  else if (h < 300) { rt = X; gt = 0; bt = C; }
  else              { rt = C; gt = 0; bt = X; }

  *r = (int)((rt+m)*255); *g = (int)((gt+m)*255); *b = (int)((bt+m)*255);
};

float norm_int(int x) {
  return (float)x / 255;
};

void Person::publish_line_strip(geometry_msgs::Point a, geometry_msgs::Point c, int body_part) {

  // if we have no detected point the value is (0,0,0) by default. therefore, we assume that
  // all points at this location is invalid.
  if ((a.x == 0.0 && a.y == 0.0 && a.z == 0.0) ||
      (c.x == 0.0 && c.y == 0.0 && c.z == 0.0)) {
    //ROS_INFO("Did not draw %s in person %d", bpnames[body_part], Person::id);
    return;
  }
  std::stringstream namesp;
  namesp << "person_" << id;
  visualization_msgs::Marker marker;

  marker.header.frame_id = base_frame;
  marker.header.stamp = ros::Time::now();

  marker.ns = namesp.str();
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;

  marker.id = body_part;//bpnames[body_part];
  marker.type = visualization_msgs::Marker::LINE_STRIP;

  marker.scale.x = 0.01;
  int r, g, b;
  hsv_to_rgb(body_part*21, 1, 1, &r, &g, &b);
  marker.color.r = norm_int(r);
  marker.color.g = norm_int(g);
  marker.color.b = norm_int(b);
  marker.color.a = 1.0;
  // TODO: test with just &maker.color.<rgb>
  marker.points.push_back(a);
  marker.points.push_back(c);

  Person::marker_pub.publish(marker);
  marker.points.clear();
};

void Person::draw_skeleton() {
  // for (int i = 0; i < sizeof(bpnames)/sizeof(bpnames[0]); i++)
  //   std::cout << bpnames[i] << std::endl;

  // for (int i = 1; i < sizeof(bpnames)/sizeof(bpnames[0]); i++)
  //   publish_line_strip(coordinates[i-1], coordinates[i], i*20);
  int r,g,b;

  // ROS_INFO("Drawing skeleton for person %d", id);
  
  publish_line_strip(coordinates[0], coordinates[1], 0);
  
  publish_line_strip(coordinates[1], coordinates[2], 1);
  publish_line_strip(coordinates[2], coordinates[3], 2);
  publish_line_strip(coordinates[3], coordinates[4], 3);
  
  publish_line_strip(coordinates[1], coordinates[5], 4);
  publish_line_strip(coordinates[5], coordinates[6], 5);
  publish_line_strip(coordinates[6], coordinates[7], 6);
  
  publish_line_strip(coordinates[1], coordinates[8], 7);
  publish_line_strip(coordinates[8], coordinates[9], 8);
  publish_line_strip(coordinates[9], coordinates[10], 9);
  
  publish_line_strip(coordinates[1], coordinates[11], 10);
  publish_line_strip(coordinates[11], coordinates[12], 11);
  publish_line_strip(coordinates[12], coordinates[13], 12);
  
  publish_line_strip(coordinates[0], coordinates[14], 13);
  publish_line_strip(coordinates[14], coordinates[16], 14);
  publish_line_strip(coordinates[0], coordinates[15], 15);
  publish_line_strip(coordinates[15], coordinates[17], 16);
  
  return;
};
