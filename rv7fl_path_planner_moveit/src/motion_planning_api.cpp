/*These codes are based on the moveit tutorial written by Sachin Chitta and Michael Lautman*/
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>

int main(int argc, char ** argv)
{
  const std::string node_name = "rv7_motion_planning_api_node";
  ros::init(argc, argv, node_name);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // Start
  // """""
  // Planners are setup as plugins in Moveit. and I use 
  // ROS pluginlib interface to load any planner. Before
  // loading a planner I need:
  // (a) a RobotModel 
  // (b) a PlanningScene
  // I instantiate a RobotModelLoader object, which
  // will look for 'robot_description' parameter from
  // ROS server and construct a moveit_core::RobotModel 
  // me to reach all the robot parameter
  // 
  const std::string PLANNING_GROUP = "arm";
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader("robot_description"));
  
  robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();
  /*I create a RobotState and JointModelGroup to keep track of 
    the robot state and planning group */
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  // I construct a planning_scene::PlanningScene using moveit_core::RobotModel.
  // that maintains the state of the world (including the robot)
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  
  //Based on mlautman merge
  //with planning scene I create a planning scene moniotor that
  //monitor planning scene diffs and apply them to the planning scene

  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(planning_scene, robot_model_loader));
  psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  psm->startStateMonitor();
  psm->startSceneMonitor();

  while(!psm->getStateMonitor()->haveCompleteState() && ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(1, node_name, "waiting for complete state from topic ");
  }


  // I construct a loader to load a planner, by name
  // Note that I am using the ROS pluginlib library.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planning_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  // I get the name of the planning plugin I want to load
  // from the ROS parameter server, and then load the planner
  // making sure to catch all exception

  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planning_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planning_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planning_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() 
        << std::endl << "Available plugin: " << ss.str());
  }
  // Visualization
  // """""""""""""
  // The package MoveItVisualTools provides many capabilities for visualizing
  // object, robots, and trajectory in RVIZ as well as debugging tools such 
  // step-by-step introspection of a scropt
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base", rviz_visual_tools::RVIZ_MARKER_TOPIC, psm); //[IMAN] or "floor"? 
  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  
  // Remote control is an introspection tool that allows users to 
  // step through a high level script via buttons and keyboards in RViz

  visual_tools.loadRemoteControl();


  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose,"Motion Planning API", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  
  //Batch publishing is used to reduce the number of messages being sent to RVIZ
  //for large visualization
  visual_tools.trigger();
  
  //I also use visual tools to wait for user input
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start");

  // Pose Goal
  // """""""""
  // I create a motion plan request for the arm of RV7FR 
  // specifying the desried pose of the end-effector as input. 
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.trigger();
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "floor";
  pose.pose.position.x = 0.6018;
  pose.pose.position.y = 0.2863;
  pose.pose.position.z = 0.3447;
  pose.pose.orientation.w = 1.0;
  /*req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = 
    req.workspace_parameters.min_corner.z = -2.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
    req.workspace_parameters.max_corner.z = 2.0; */


  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
  
  // We will create the request as a constraint using a helper
  // function available from the 
  // kinematic_constraints package
  // http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  //

  //req.group_name = "arm";
  moveit_msgs::Constraints pose_goal = 
    kinematic_constraints::constructGoalConstraints("link_effector", pose, tolerance_pose, tolerance_angle);
  
  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);

  // I now construct a planning context that encapsulate the scene,
  // the request and the response. I call the planner using this 
  // planning context 
  planning_interface::PlanningContextPtr context = 
    planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  // Visualize the result
  // """"""""""""""""""""
  ros::Publisher display_publisher =
    node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);
  
  //Set the state in the planning scene to the final state of the last plan
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // Display the goal state
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.publishAxisLabeled(pose.pose, "goal_1");
  visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'Next'");
  
  // Joint Space Goals
  // """""""""""""""""
  // First, set the state in the planning scene to the final state
  // of the last plan
  // Now, setup a joint space goal
  robot_state::RobotState goal_state(robot_model);
  std::vector<double> joint_values = { -0.0182, -0.2585, -0.1743, -0.3680, 0.3483, 0.3139 };
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // Call the planner and visualize the trajectory
  // Re-construct the planning context
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  /* Call the planner */
  context->solve(res);
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);

  /* Now I should see two planned trajectories in series */
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);
  /* I add more goals. But first, I set the state in the planning
   * scene to the final state of the last plan */
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // Display the goal state
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.publishAxisLabeled(pose.pose, "goal_2");
  visual_tools.publishText(text_pose, "Joint Space Goal (2)", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /*Wait for user input */
  visual_tools.prompt("Press 'Next'"); 
  /* Now I go back to the first goal */
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);

  // Set the state in the planning scene to the final state of the last plan
  robot_state->setJointGroupPositions(
      joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.trigger();

  // Display the goal state
  visual_tools.prompt("Press 'Next'");


  // Adding Path contraints
  // """"""""""""""""""""""
  // I add a new pose goal again. This time I will also add a path constraint
  // to the motion. 
  pose.pose.position.x = 0.5092;
  pose.pose.position.y = -0.1599;
  pose.pose.position.z = 0.3989;
  pose.pose.orientation.w = 1.0;
  moveit_msgs::Constraints pose_goal_2 = 
    kinematic_constraints::constructGoalConstraints("link_effector", pose, tolerance_pose, tolerance_angle);
  /* First, set the state in the planning scene to the final state of the last plan */
//  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  /* Now, let's try to move to this new pose goal */
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal_2);

  geometry_msgs::QuaternionStamped quaternion;
  quaternion.header.frame_id = "base";
  quaternion.quaternion.w = 1.0;
  req.path_constraints = kinematic_constraints::constructGoalConstraints("link_effector", quaternion);

  // Imposing path contraints requires the planner to reason in the space 
  // of possible positions of the end-effector
  // (the workspace of the robot) because of this, I need to 
  // specify a bound for the allowed planning volume as well;
  // Note: a default bound is automatically filled by the WorkspaceBounds request
  // addapter (part of the OMPL pipeline)
  // but tat is not being used here for now. 
  // I use a bound that definitely includes the reachable space for the arm. This if fine
  // because sampling is not done in this volume when planning
  // for the arm; the bound are only used to determine if the sampled 
  // configuration are valid
  // Call the planner and visualize all the plans created so far.
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = 
    req.workspace_parameters.min_corner.z = -2.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = 
    req.workspace_parameters.max_corner.z = 2.0;

  
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  /* Now, four planned-trajectories are shown in series */
  display_publisher.publish(display_trajectory);

  //Set the state in the planning scene to the final state of the last plan
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // Display the goal state 
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.publishAxisLabeled(pose.pose, "goal_3");
  visual_tools.publishText(text_pose, "Orientation Constraint Motion Plna (3)", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // END_TUTORIAL
  /* Wait for user input */
  visual_tools.prompt("Press 'Next' to exit demo");
  planner_instance.reset();
  
  return 0;
}


  



  



























