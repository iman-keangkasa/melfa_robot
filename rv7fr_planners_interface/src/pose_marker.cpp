#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <math.h>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;


void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
  /*
  if(feedback->pose.position.y > 1.0) 
  {
    server->clear();

    tf2::Vector3 feedback_position = tf2::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
    tf2::Quaternion feedback_orientation = tf2::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w);

    //make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, feedback_position , feedback_orientation, true, false); 
    //ROS_WARN_STREAM("Change appearance");
    call_me();
  }
  */
  server->applyChanges();
}

Marker makeBox( InteractiveMarker &msg, bool green=true)
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  
  if(green)
  {
    marker.color.r = 0;
    marker.color.g = 1.0;
    marker.color.b = 0;
    marker.color.a = .7;
  }
  else
  {
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 0.7;
  }
  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg, bool green=true)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg,green) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void make6DofMarker( bool fixed, 
                      unsigned int interaction_mode, 
                      const tf2::Vector3& position,
                      const tf2::Quaternion& orientation,
                      bool show_6dof,
                      bool green=true
                      )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf2::toMsg(position, int_marker.pose.position);
  int_marker.pose.orientation = tf2::toMsg(orientation);
  int_marker.scale = 1;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker,green);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  
}



int main( int argc, char** argv)
{
  ros::init(argc, argv, "pose_marker");
  ros::NodeHandle n;
  
  server.reset(new interactive_markers::InteractiveMarkerServer("pose_marker","", false));
/*  
  tf2::Vector3 pose_d;
  pose_d = tf2::Vector3(0.,0.,0.);
  
  tf2::Quaternion orien_d;
  orien_d = tf2::Quaternion(0.,0.,0.,1.);
*/  
  tf2::Vector3 position;
  position = tf2::Vector3(0,0,0);
  tf2::Quaternion orientation(0,0,0,1);
  orientation.normalize();
  make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, orientation, true);

  server->applyChanges();
  ros::spin();
  server.reset();
  return 0;
}

