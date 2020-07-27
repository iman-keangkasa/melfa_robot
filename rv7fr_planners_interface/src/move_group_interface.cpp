#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  //initialize node and its name
  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setting Up 
  // Moveit Operates on sets of joints; planning_groups and stores them
  // in an object called the 'JointModelGroup'.
  static const std::string PLANNING_GROUP = "arm";
  
  //Here the planning_intrface::MoveGroupInterface Class is being set up
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  //To add and remov a collision objects in the virtual world scene
  // we will use the planning_interface::PlanningSceneInterface class
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  // We always pointers to improve performance
  const moveit::core::JointModelGroup* joint_model_group = 
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualize
  namespace rvt = rviz_visual_tools; 
  moveit_visual_tools::MoveItVisualTools visual_tools("table");
  visual_tools.deleteAllMarkers();
  
  //Remote control
  visual_tools.loadRemoteControl();

  //RVIZ markers
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.tranlaste().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface", rvt::WHITE, rvt::XLARGE);

  // Bathc publishing to reduce message
  visual_tools.trigger();

  // Getting basic information
  ROS_INFO_NAMED("RV7FR", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("RV7FR", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("RV7FR", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group,getJointModelGroupNames.end(),
      std::ostream_iterator<std::string>(std::cout, ", "));
  
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.7071915;
  target_pose1.orientation.y = 0;
  target_pose1.orientation.z = 0.707022;
  target_pose1.orientation.w = 0;
  target_pose1.position.x = 0.610798;
  target_pose1.position.y = -0.36939;
  target_pose1.position.z = 1.06357;
  move_group.setPoseTarget(target_pose1);
  
  //Call the planner to compute the plan and visualize it
  moveit::planning_interface::MoveGroupInterface::Plan plan1; 
  bool success = (move_group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("RV7FR", "Visualizing Plan 1 (pose goal) %s", success ? ""; "FAILED");

  //Visualizing plans
  ROS_INFO_NAMED("RV7FR", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(plan1.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Finish visualizing Plan 1, press Next to continue");
  
  ros::shutdown();
  return 0;
}


  






















































