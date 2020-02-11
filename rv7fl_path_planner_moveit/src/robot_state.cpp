/************************************
 * The robot state of RV7 series
 * This code is based on the tutorial 
 * written by Sachin Chitta
 ***********************************/

#include <ros/ros.h>

//Moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main( int argc, char ** argv)
{
  ros::init(argc, argv, "robot_state");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //I use RobotModel class with shared pointer. I use
  //shared pointer as much as possible. 
  //
  //I instantiate a 'RobotModelLoader' object.
  //The object will look for the robot_description
  //parameter from the ROS parameter server
  //and constuct the a moveit_core: 'RobotModel' 
  //to access the robot state. The robot model is named as
  //kinematic_model

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  
  //I use moveit_core:'RobotModel' object to construct moveit_core:'RobotState
  //that maintains the configuration of the robot. I set all the joints in the
  //state to their default values. I can get the moveit_core:'JointModelGroup; 
  //which represents the robot model for a particular group

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Get Joint Values
  // """"""""""""""""
  // I retrieve the current set of joint values stored in the state of the
  // rv7 arm

  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  
  //Joint Limits
  //""""""""""""
  // setJointGroupPositions() 
  joint_values[0] = 5.57;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
  //Check if joints are outside joint limits
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  // Forward Kinematics
  // """"""""""""""""""
  //
  // I compute the forward kinematics for a set of random joint values
  // Note that I would like to find the pose of the most distal link
  // in the "arm" of rv7 
  kinematic_state->setToRandomPositions(joint_model_group); //[IMAN] get from current 
  const Eigen::Affine3d & end_effector_state = kinematic_state -> getGlobalLinkTransform("link_effector");

  // Print end-effector pose
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n"); 

  // Inverse Kinematics
  // """"""""""""""""""
  // I solve the inverse kinematics for the RV7 by:
  // - Defining the desired pose of the end-effector 
  // - The number of attempts to solve the IK: 10
  // - The timeout for each attempt: 0.1 s

  std::size_t attempts = 10;
  double timeout = 0.1;
  bool found_ik = kinematic_state -> setFromIK(joint_model_group, end_effector_state, attempts, timeout);

  //I print out the IK solution (if found):
  if(found_ik)
  {
    kinematic_state -> copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  // Get the Jacobian
  // """"""""""""""""
  // I get the Jacobian from the moveit_core: RobotState
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state -> getJacobian(joint_model_group, 
      kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), 
      reference_point_position, jacobian);
  ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

  ros::shutdown();
  return 0;
}













































