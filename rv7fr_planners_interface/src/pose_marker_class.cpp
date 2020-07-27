#include <ros/ros.h>
/*
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
*/
#include <boost/thread/mutex.hpp>
#include <math.h>

#include "rv7fr_planners_interface/pose_marker_class.h"

PoseMarker::PoseMarker(std::string ns) :
  ik_start_found_("false"),
  ik_end_found_("false"),
  server_(ns,"",false),
  start_str_("start"),
  end_str_("end"),
  robot_model_loader_("robot_description"),
  test_string("Heyah")
{
  kinematic_model_ = robot_model_loader_.getModel();
  kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup("arm"); 
  joint_model_group_ = joint_model_group;
  
  text_position_ = tf2::Vector3(1,1,1);
  
  start_.position.x = 0.47;
  start_.position.y = 0.47;
  start_.position.z = 1.00;
  start_.orientation = tf2::toMsg(tf2::Quaternion(0.0, 1.0, 0.0, 1.0));

  end_.position.x = 0.47;
  end_.position.y = -0.47;
  end_.position.z = 1.00;
  end_.orientation = tf2::toMsg(tf2::Quaternion(0.0, 1.0, 0.0, 1.0));
  last_valid_start_ = start_;
  last_valid_end_ = end_; 
  /*
  int_position_ = tf2::Vector3(1,1,1);
  int_orien_ = tf2::Quaternion(0,0,0,1);
  int_orien_.normalize();
  */
  //setPose(start_str_);
  //setPose(end_str_);

  make6DofMarker();
  server_.applyChanges();
  ns_ = ns;
  //pub_ = n_.advertise<geometry_msgs::Pose>( ns+"/"+start_end_, 1, true);
}

/*
void setPose(const std::string& start_end)
{
  if(start_end == "start")
  {
    start_.position.x = 0.47;
    start_.position.y = 0.47;
    start_.position.z = 1.00;
    start_.orientation = tf2::toMsg(tf2::Quaternion(0.0, 1.0, 0.0, 1.0));
  }
  else
  {
    end_.position.x = 0.47;
    end_.position.y = -0.47;
    end_.position.z = 1.00;
    end_.orientation = tf2::toMsg(tf2::Quaternion(0.0, 1.0, 0.0, 1.0));
    
  }  
}
*/

void PoseMarker::printPoseMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::ostringstream& pose_text)
{
  InteractiveMarker int_text_;
  int_text_.header.frame_id="world";
  int_text_.name = "pose_information";
  InteractiveMarkerControl control; 
  control.always_visible = true;

  Marker marker;
  marker.type = Marker::TEXT_VIEW_FACING;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.text = pose_text.str();
  marker.pose.position.x = feedback->pose.position.x; //[IMAN]
  marker.pose.position.y = 0.25 + feedback->pose.position.y; //[IMAN]
  marker.pose.position.z = 0.25 + feedback->pose.position.z;
  control.markers.push_back( marker );
  int_text_.controls.push_back( control );
  server_.insert( int_text_ );

}

void PoseMarker::processFeedback( std::string start_end_, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s_header;
  s_header << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";
  
  std::ostringstream pose_textg;
   
  ROS_INFO_STREAM( s_header.str() << ": pose changed" << pose_textg.str()
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
  
  //robot_model_loader::RobotModelLoader robot_model_loader;
  //moveit::core::RobotModelPtr kinematic_model; 
  //moveit::core::RobotStatePtr kinematic_state( new moveit::core::RobotState(kinematic_model) );
  std::ostringstream ik_valid("");

  
  if( kinematic_state_->setFromIK(joint_model_group_, feedback->pose, 0.0) ) 
  {
    
    ik_valid << "IK FOUND";
    ROS_WARN_STREAM(ik_valid.str());
    pub_.publish(feedback->pose);
    if (start_end_ == start_str_) 
    {
        start_ = feedback->pose;
        ik_start_found_ = true;
    }
    else
    {
        end_ = feedback->pose;
        ik_end_found_ = true;
    }
  }
  else
  {
    if (start_end_ == start_str_) 
    {
        start_ = feedback->pose;
        ik_start_found_ = false;
    }
    else
    {
        end_ = feedback->pose;
        ik_end_found_ = false;
    }
  }

  //tf2::fromMsg(feedback->pose.position, int_position_);
  //tf2::fromMsg(feedback->pose.orientation, int_orien_);
  
  std::string topic_name (ns_ + "/" + start_end_);
  pose_textg << ik_valid.str()
          << "\n" << topic_name <<  " position"
          << "\nx: " << feedback->pose.position.x
          << "\ny: " << feedback->pose.position.y 
          << "\nz: " << feedback->pose.position.z
          << "\n" << topic_name << " orientation"
          << "\nx: " << feedback->pose.orientation.x  
          << "\ny: " << feedback->pose.orientation.y 
          << "\nz: " << feedback->pose.orientation.z
          << "\nw: " << feedback->pose.orientation.w;
  ROS_INFO_STREAM(pose_textg.str());
  printPoseMarker(feedback, pose_textg);
  //server_.applyChanges();
  //server_.clear();
  //make6DofMarker(); 
  server_.applyChanges();
}

void PoseMarker::makeBox(const std::string& name)
{
  //[IMAN] Make this general later
  box_.type = visualization_msgs::Marker::MESH_RESOURCE;
  box_.mesh_resource = "package://melfa_description/mesh/rv7fr/dh_link6.stl";
  box_.scale.x = 0.001;
  box_.scale.y = 0.001;
  box_.scale.z = 0.001;
  
  if(name == "start")
  {
    box_.color.r = 0;
    box_.color.g = 1.0;
    box_.color.b = 0;
    box_.color.a = .7;
  }
  else
  {
    box_.color.r = 2;
    box_.color.g = 0;
    box_.color.b = 0;
    box_.color.a = 0.7;
  } 
}

void PoseMarker::makeBoxControl(const std::string& name, InteractiveMarker& int_marker_)
{
  
  int_marker_.header.frame_id = "world";
  
  if(name == start_str_)
  {
      int_marker_.pose = start_;
  }
  else
  {
      int_marker_.pose = end_;
  }

  int_marker_.scale = 0.2;
  int_marker_.name = name;
  int_marker_.description = "";

  InteractiveMarkerControl int_control_;
  int_control_.always_visible = true;
  makeBox(int_marker_.name);
  int_control_.markers.push_back( box_  );
  int_control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  int_marker_.controls.push_back( int_control_ );

  InteractiveMarkerControl int_control2_;
  tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
  orien.normalize();
  int_control2_.orientation = tf2::toMsg(orien);
  int_control2_.name = "rotate_x";
  int_control2_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(int_control2_);
  int_control2_.name = "move_x";
  int_control2_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(int_control2_);

  orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  int_control2_.orientation = tf2::toMsg(orien);
  int_control2_.name = "rotate_z";
  int_control2_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(int_control2_);
  int_control2_.name = "move_z";
  int_control2_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(int_control2_);

  orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
  orien.normalize();
  int_control2_.orientation = tf2::toMsg(orien);
  int_control2_.name = "rotate_y";
  int_control2_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(int_control2_);
  int_control2_.name = "move_y";
  int_control2_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(int_control2_);

}

void PoseMarker::make6DofMarker()
{
  InteractiveMarker int_marker_start;
  InteractiveMarker int_marker_end;
  makeBoxControl(start_str_, int_marker_start);
  makeBoxControl(end_str_,int_marker_end);
  //int_marker_.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // make control visualization
  server_.insert(int_marker_start);
  server_.setCallback(int_marker_start.name, boost::bind(&PoseMarker::processFeedback, this, start_str_, _1));
  server_.insert(int_marker_end);
  server_.setCallback(int_marker_end.name, boost::bind(&PoseMarker::processFeedback, this, end_str_, _1));

  //Create the layout of the menu
  initMenu();

  //Apply the layout to markers
  menu_handler_.apply( server_, int_marker_start.name );
  menu_handler_.apply( server_, int_marker_end.name );
  server_.applyChanges();
}

void PoseMarker::setPoseCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  //ROS_INFO_STREAM("SUCCESS!");
  if(feedback->marker_name == start_str_)
  {
    if(ik_start_found_)
    {
      last_valid_start_ = start_;
    }
    else
    {
      ROS_WARN_STREAM("IK HAS NOT BEEN FOUND AT THIS POSE");
    }
  }
  else if(feedback->marker_name == end_str_)
  {
    if(ik_end_found_)
    {
      last_valid_end_ = end_;
    }
    else
    {
      ROS_WARN_STREAM("IK HAS NOT BEEN FOUND AT THIS POSE");
    }
  }
}

geometry_msgs::Pose PoseMarker::getStart()
{
  return last_valid_start_;
}

geometry_msgs::Pose PoseMarker::getEnd()
{
  return last_valid_end_;
}


void PoseMarker::initMenu()
{
  menu_handler_.insert( "Set this pose", boost::bind(&PoseMarker::setPoseCb, this, _1) );
} 

