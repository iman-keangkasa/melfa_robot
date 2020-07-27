#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoidance");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = 
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  //Visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  visual_tools.loadRemoteControl();

  
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;


  geometry_msgs::Pose start_pose;
  start_pose.orientation.x = -0.707083360325;
  start_pose.orientation.y = 5.17992987498e-05;
  start_pose.orientation.z = -0.707130199162;
  start_pose.orientation.w = 1.73810759282e-05;
  start_pose.position.x = 0.454529053894;
  start_pose.position.y = -2.21256983054e-05;
  start_pose.position.z = 1.74503743222;

  geometry_msgs::Pose pose1;
  pose1.orientation.x = -0.707214994375;
  pose1.orientation.y = -4.57488265851e-5;
  pose1.orientation.z = -0.706998495513;
  pose1.orientation.w = 0.000277452732221;
  pose1.position.x = 0.59070049247;
  pose1.position.y = 0.326491323297;
  pose1.position.z = 1.00491380104;

  geometry_msgs::Pose pose2;
  pose2.orientation.x = 0.707290371072;
  pose2.orientation.y = -8.81092724885e-5;
  pose2.orientation.z = 0.706923134122;
  pose2.orientation.w = 7.52963067658e-5;
  pose2.position.x = 0.591001115432;
  pose2.position.y = -0.31478017123;
  pose2.position.z = 1.00483301414;
/*
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "link_effector";
  ocm.header.frame_id = "world";
  ocm.orientation.x = -0.707214994375;
  ocm.orientation.y = -4.57488265851e-5;
  ocm.orientation.z = -0.706998495513;
  ocm.orientation.w = 0.000277452732221;
  ocm.absolute_x_axis_tolerance = 0.005;
  ocm.absolute_y_axis_tolerance = 0.005;
  ocm.absolute_z_axis_tolerance = 0.005;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

*/

  move_group.setPoseTarget(pose1);
  
  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  bool success = (move_group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("RV7FR", "Pose Goal Plan 1 %s", success ? "" : "FAILED");
  
  if(success) move_group.execute(plan1.trajectory_);
  

  //Obstacle setting
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.id = "cylinder";
  
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions = {1.4, 0.2};

  geometry_msgs::Pose cylinder_pose;
  cylinder_pose.orientation.w = 1.0;
  cylinder_pose.position.x = 0.55;
  cylinder_pose.position.y = 0;
  cylinder_pose.position.z = 0.7;
  
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cylinder_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface.addCollisionObjects(collision_objects);

  move_group.setPoseTarget(pose2);
  success = (move_group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("RV7FR", "Pose Goal Plan 2 %s", success ? "" : "FAILED");
  if(success) move_group.execute(plan1.trajectory_);

  ros::shutdown();
  return 0;
}






