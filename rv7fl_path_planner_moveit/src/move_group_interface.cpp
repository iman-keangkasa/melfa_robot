/* Author: Hafiz Iman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main (int argc, char ** argv)
{
  ros::init(argc, argv, "rv7_move_group_interface_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //we get a planning object. To do this I need my moveit config
  //for the right robot in IIUM Robotics Lab
  static const std::string PLANNING_GROUP = "arm";

  //I initialize the MoveGroup class with the name of the
  //arm I am planning for 
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  //I use the planning_scene_interface (an object of calss PlanningSceneInterface)
  //to add and remove collision objects in the workspace 
  //of the my robot
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //I use raw pointers to improve performance
  //joint_model_group is a pointer to a class
  const robot_state::JointModelGroup * joint_model_group = 
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  //Visualization
  //"""""""""""""
  //
  //The package MoveItVisualTools provides capabilities to visualize 
  //objects, robots and trajectories in RVIZ, and debugging tools 

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base"); 
  visual_tools.deleteAllMarkers();

  //Remote control to aide visulization 
  //using GUI such as buttons and keyboard shortcuts
  visual_tools.loadRemoteControl();

  //Rviz has many markers 
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75; // meter
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  //Getting Basic information
  //"""""""""""""""""""""""""
  //I print the name of the reference frame for this robot
  ROS_INFO_NAMED("Move Group", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  //I print the name of the end-effector link for this group.
  ROS_INFO_NAMED("Move Group", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  visual_tools.prompt("Press next in the RvizVisualToolsGui window to start move group");

  //Planning to a pose goal
  //"""""""""""""""""""""""

  //We can plan a motion for this group to a desired pose for the end 
  //effector 
  geometry_msgs::Pose target_pose1; //[IMAN] Please change these values before building
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.4;
  move_group.setPoseTarget(target_pose1);

  //Now, I call the planner to compute the plan and visualize it
  //Here I am just calling the planner and not asking the
  //move_group to move the robot
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("Move Group", "Visualizing plan 1 (pose goal) %s", success? "": "FAILED");
  //Visualizing plans
  //"""""""""""""""""
  //I now will visualize the line of the plan with the markers class in RViz.
  //
  ROS_INFO_NAMED("Move Group", "Visualizing plan as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  //Moving to pose goal
  //"""""""""""""""""""
  //
  //Moving to a pose goal requires a physical robot
  //This is the robotHw interface for the moveit that
  //integrate ros_control
  //
  //[IMAN] try and do this with gazebo
  /* move_group.move(); */

  //Planning to a joint-space goal
  //""""""""""""""""""""""""""""""
  //
  //I move the robot in the joint space.
  //This will replace the pose target I set earlier

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  //Next get the current set of joint values for the group
  std::vector <double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


  //I now change one of the joints, plan to the new joint 
  //and visualize the plan
  joint_group_positions[0] = -1.0; //radian
  move_group.setJointValueTarget(joint_group_positions);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("Move group", "Visualizing plan in joint space %s", success ? "" : "Failed");

  //Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press Next in RVizVisualToolGui window to continue");
  
  //Planning With Path Constraints
  //""""""""""""""""""""""""""""""
  //
  //Path constraints can be easily specified for a link on the robot
  //I specify a path constraint and a pose goal for my group
  //
  //I first define the path contraint
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "link_x"; //[IMAN] change this
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  //I set the path contraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  //I use the old goal that I plan with to demonstrate
  //the constraint. This only work if the current state
  //already satisfies the path constraints. So, I need to 
  //set the start state to a new pose.
  robot_state::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  //I  plan to the earlier pose target from the new start
  //position above
  move_group.setPoseTarget(target_pose1);

  //Planning with constraint can be slow because every sample must call
  //an inverse kinematics solver.

  //I increase the planning time from the default 5 seconds to
  //make sure the planner has enough time to find 
  //a trajectory
  move_group.setPlanningTime(10.0);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //I visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  //When done with the path constraint be sure to clear it
  move_group.clearPathConstraints();

  //Cartesian Paths
  //"""""""""""""""
  //
  //I plan using cartesian motion by specifying a list of 
  //waypoints for the end effector to go through. I am starting from
  //the new start state above (start_state)
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);
  geometry_msgs::Pose target_pose3 = start_pose2;

  target_pose3.position.z -= 0.2; //[IMAN] Change base on robot
  waypoints.push_back(target_pose3); //down 20cm
  
  target_pose3.position.y -= 0.2; //[IMNA] Change base on robot
  waypoints.push_back(target_pose3);

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3); //up and left

  //I scale down joint rotational speed of each joints
  move_group.setMaxVelocityScalingFactor(0.1);

  //I would want to the Cartesion path to be interpolated at a resolution of 1 cm
  //Which is why I would specify 0.01 as the max step in Cartesian translation
  //I will specify the jump threshold as 0.0, effectively disabling it.
  //This a warn, disabling the jump threshold while operating real hardware
  //can cause large unpredictable motions of redundant joints and colud be a
  //safety issue

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eff_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eff_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("Move group interface", "Visualizing Cartesian Path (%0.2f%% achieved)", fraction * 100.0);

  //Visualizing the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i],  "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to continue");

  //Adding/Removing Objects and Attaching/Detaching Objects
  //"""""""""""""""""""""""""""""""""""""""""""""""""""""""

  //Define a collision onject ROS message
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();
  
  //The ID of the object is used to identify it
  collision_object.id = "box1";
  
  //Define a box to add to the world
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  //Define a pose for the box relative to the frame_id
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0; //[IMAN] Change this value according to robot
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  
  //I add the collision object into the world
  ROS_INFO_NAMED("Scene Interface", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  //I note this action in rviz
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  //I wait for MoveGroup to receive and process the collision object message
  visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window once the collision object appreas in RViz");

  //Now when I plan a trajectory it will avoid the obstacle
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.w = 1.0; //[IMAN] Change these values according to robot
  another_pose.position.x = 0.4;
  another_pose.position.y =  -0.4;
  another_pose.position.z = 0.9;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("Move Group", "Visualizing plan 5: Pose goal move around cuboid) %s", success? "": "FAILED");

  //Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  //Now, I attache the collision object to the robot.
  ROS_INFO_NAMED("Move Group", "Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  //I show text in RViz as status
  visual_tools.publishText(text_pose, "Object atttached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  //I wait for the MoveGroup to receive and process the attached collison object message
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the collision object attaches to the robot");

  //Now, I detach the collision object from the robot
  ROS_INFO_NAMED("Move Group", "Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  //text Status
  visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);

  //Wait for MoveGroup to receive and proceess the object message
  visual_tools.prompt("Press 'next' in the RVizVisualToolsGui Window to once the collision object disappears");

  ros::shutdown();
  return 0;
}









  











































