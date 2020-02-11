#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

//this program is based on a tutorial written by Sachin Chitta
//and Michael Lautman

//I can specify my planning scene and define my constraints
//using PlanningScene class. This is done by specifying a callback
//using the setStateFeasibilityPredicate function. Here I wrote 
//a simple example of a user-defined callback that checks
//if joint1 of Rv7 robot, using the arm move group, is at 
//a positive or negative angle:

bool stateFeasibilityTestExample(const robot_state::RobotState& kinematic_state, bool verbose)
{
  const double* joint_values = kinematic_state.getJointPositions("joint1");
  return (joint_values[0] > 0.0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::size_t count = 0;

  // Setup
  // """""
  //
  // The planning_scene: PlanningScene class can be easily setup and configured 
  // using a moveit_core: RobotModel or a URDF and SRDF. However
  // this method is not recommended when instatiate a PlanningScene. 
  // The planning_scene_monitor: PlanningSceneMonitor is the recommended
  // method to create and maintatin the current planning scene (RE: planning_scene2.cpp)
  // I will instantiate a PlanningScene class directly here but I only do so to
  // show a more illustrative example

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  //Collision Checking
  //""""""""""""""""""
  //
  //Self-collision checking
  //"""""""""""""""""""""""
  //
  //First, I check the current state of the robot if it is
  //in a configuration that collides with other parts of itself.
  //I construct these objects:
  // - collision_detection_struct: CollisionRequest
  // - collision_detection_struct: CollisionResult
  //I will pass them into the collision checking function. The result is
  //contained within the CollisionResult object. Self collision checking
  //uses an 'unpadded' version of the robot. i.e. it uses the collision
  //meshes from URDF with no extra padding.
  //

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");


  //Change the state
  //""""""""""""""""
  //
  //I change the current state of the robot. The planning
  //scene maintains the current state internally.
  //I can get a referecnce to it and
  //change it and then check for collision for the
  //new robot configuration. Note in particular
  //that I need to clear the collision result before 
  //making a new collision checking request. 

  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
  //Getting Contact Information
  //"""""""""""""""""""""""""""
  //
  //First, manually set the RV7 arm to a position where I know
  //internal collision do happen. Note that this state is now
  //actually outside the joint limits, which I can also 
  //check directly
  //  
  //

  std::vector<double> joint_values = {0.0, -0.0770877024023, 0.244436852404, 0.0, 0.0, 0.0};
  const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("arm");
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  ROS_INFO_STREAM("Test 4: Current state is "
                  << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not_valid"));

  //  Now, we get contatct information for any collision that
  //  might happened at a given configuration of the arm.
  //  We set the field for contact and specify the maiximum
  //  number of contacts to be returned as a large number

  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " self collsion");
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
  {
    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
  }

  // Modyfying the Allowed Collision Matrix
  // """"""""""""""""""""""""""""""""""""""
  //
  // The collision_detection_class: AllowedCollisionMatrix (ACM)
  // provides a mechanism to tell the collision world to ignore
  // collisions between certain object: both parts of the robot and
  // objects in the world. I can tell the collision checker to
  // ignore all collision between links reported above 
  // i.e. even though the links are actually in collision, the 
  // collision checker will ignore those collisions and return 
  // not in collsion for thes particular state of the robot.
  //
  // Note also in this example how I make copies of both the
  // allowed collision matrix and the current state and passing
  // them into the collision checking function.

  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  robot_state::RobotState copied_state = planning_scene.getCurrentState();

  collision_detection::CollisionResult::ContactMap::const_iterator it2;
  for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
  {
    acm.setEntry(it2->first.first, it2->first.second, true);
  }
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
  ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  // Full Collision Checking
  // """""""""""""""""""""""
  //
  // While I have been checking for self-collision, I can use 
  // the checkCollision functions instead which will check for 
  // both self-collision and for collisions with the environment
  // (which is currently empty). This is the set of collision checking
  // functions that I will use most often in a planner. Note that collision
  // checks with the environment will use the padded version
  // of the robot. Padding helps in keeping the robot further away
  // from obstacles in the environment
  //
  //
  collision_result.clear();
  planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
  ROS_INFO_STREAM("Test 7: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  // COnstrain Checking
  // """"""""""""""""""
  //
  // The PlanningScene class also includes easy to use function calls
  // for checking contraints. The constraints can be of two types:
  //  (a) constraints chosen from the
  //    - kinematic_constraints: KinematicConstraint set
  //      - kinematic_constraints: JOintConstraint
  //      - kinematic_constraints: PositionConstraint
  //      - kinematic_constraints: OrientationConstraint
  //      - kinematic_constraints: VisibilityConstraint
  //
  //  (b) user defined constraints specified through a callback. 
  //  I will first look at an example with a simple KinematicConstraint
  //
  //  Checking Kinematic Constraints 
  //  """"""""""""""""""""""""""""""
  //
  //  I will first define a simple position and orientation constraint
  //  on the end-effector of the arm group of the RV7. NOte the use of the 
  //  convenience functions for filing up the constraints
  //  (these functions are found in the moveit_core_files: utils.h<utils_8h>
  //  file from the kinematic_contraints directory in moveit_core
  //
  
  std::string end_effector_name = joint_model_group->getLinkModelNames().back();
  
  geometry_msgs::PoseStamped desired_pose;
  desired_pose.pose.orientation.w = 1.0;
  desired_pose.pose.position.x = 1.0;
  desired_pose.pose.position.y = 1.0;
  desired_pose.pose.position.z = 1.0;
  desired_pose.header.frame_id = "world";
  moveit_msgs::Constraints goal_constraints = 
    kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);

  //Now, I will check a state against this constraint using the 
  // isStateContrained functions in the PlanningScene class.

  copied_state.setToRandomPositions();
  copied_state.update();
  bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraints);
  ROS_INFO_STREAM("Test 8: Random state is " << (constrained ? "contrained" : "not constrained"));

  //There is a more efficient way of checking constraints 
  //(when you want to check the same constraint over and over 
  //again, e.g. inside a planner). I first construct a KinematicConstraintSet
  //which pre-processes the ROS constraints messages and set it up
  //for quick processing
  
  kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
  kinematic_constraint_set.add(goal_constraints, planning_scene.getTransforms());
  bool constrained_2 = planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
  ROS_INFO_STREAM("Test 9: Random state is " << (constrained_2 ? "constrained" : "not_constrained"));

  //There's a direct way to do this using the KinematicConstrainSet Class

  kinematic_constraints::ConstraintEvaluationResult constraint_eval_result = 
    kinematic_constraint_set.decide(copied_state);
  ROS_INFO_STREAM("Test 10: Random state is " 
      << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));

  // User-defined constraints
  // """"""""""""""""""""""""
  //
  // CALL the callback function above
  //
  // Now, whenever isStateFeasible is called, this user-defined
  // callback will be called

  planning_scene.setStateFeasibilityPredicate(stateFeasibilityTestExample);
  bool state_feasibility = planning_scene.isStateFeasible(copied_state);
  ROS_INFO_STREAM("Test 11: Random state is " << (state_feasibility ? "feasible" : "not feasible"));

  // Whenever isStateValid is called, three checks are conducted:
  // (a) collision checking
  // (b) constraint checking
  // (c) feasibility checking using the user-defined callback

  bool state_valid = planning_scene.isStateValid(copied_state, kinematic_constraint_set, "arm");
  ROS_INFO_STREAM("Test 12: Random state is " << (state_valid ? "valid" : "not valid" ));

  //Note that all the planners available through MoveIt! and OMPL will currently
  //perform collision checking, constrain checking and feasibility checking 
  //using the user-defined callbacks.

  ros::shutdown();
  return 0;
}





















































