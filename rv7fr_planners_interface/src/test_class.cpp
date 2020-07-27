#include <ros/ros.h>
#include "rv7fr_planners_interface/pose_marker_class.h"

int main( int argc, char** argv)
{
  std::string pose_marker("class_pose");
  ros::init(argc, argv, pose_marker);
  PoseMarker marker1(pose_marker);
  ROS_WARN_STREAM(marker1.test_string);
  ros::spin();
  return 0;
} 
