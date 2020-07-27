#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <map>
//#include <interactive_markers/interactive_marker_server.h>

#ifndef POSE_MARKER_H
#define POSE_MARKER_H

using namespace visualization_msgs;

class PoseMarker
{
  public:
    PoseMarker(std::string ns);
    //void setPose(const std::string& start_end);
    void printPoseMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::ostringstream& pose_text ); 
    void processFeedback( std::string start_end_, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    void makeBox(const std::string& name);
    void makeBoxControl(const std::string& name, InteractiveMarker& int_marker_);
    void make6DofMarker();
    void setPoseCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    geometry_msgs::Pose getStart();
    geometry_msgs::Pose getEnd();
    void initMenu();
    std::string test_string; 
  private:
    interactive_markers::InteractiveMarkerServer server_;
    interactive_markers::MenuHandler menu_handler_;
    std::ostringstream pose_text_;
    //std::string start_end_;
    std::string ns_;
    std::string start_str_;
    std::string end_str_;
    geometry_msgs::Pose start_;
    geometry_msgs::Pose end_;
    geometry_msgs::Pose last_valid_start_;
    geometry_msgs::Pose last_valid_end_;
    //InteractiveMarker int_marker_;
    //InteractiveMarker int_text_;
    //InteractiveMarkerControl int_control_;
    //InteractiveMarkerControl int_control2_;
    visualization_msgs::Marker box_; 
    bool ik_start_found_;
    bool ik_end_found_;
    tf2::Vector3 text_position_;
    //tf2::Vector3 int_position_;
    //tf2::Quaternion int_orien_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::ServiceServer service_;
    std::map<std::string, int> m_;

   
    
};

#endif
