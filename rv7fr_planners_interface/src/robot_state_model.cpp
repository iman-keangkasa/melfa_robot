#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

//For converting Isometry3d into message object I 
//would use tf2_eigen/tf2_eigen.h 
//
//There are two namespaces in this header file
//tf2 and Eigen. Eigen has all the function
//tf2 has classes 
#include <tf2_eigen/tf2_eigen.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "robot_state_model");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  //Make sure I have loaded the robot urdf into robot_description
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description"); //load robot description
  moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model Frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues(); //Not sure what this does
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
  
  //Get joint names and joint values
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  
  //Joint limit
  joint_values[2] = 3.143;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  kinematic_state->enforceBounds(); //enforce bound: for invalid values, it is inforced to be valid
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
  
  // Forward Kinematics

  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("link_effector");
  
  //std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  geometry_msgs::Pose ee_pose_msg = Eigen::toMsg(end_effector_state); 
  
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation().x() << "\n");
  ROS_INFO_STREAM("Rotation:\n" << end_effector_state.rotation() << "\n");

  // Inverse Kinematics
  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  //Get the jacobian
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                                kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                reference_point_position, jacobian);
  ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");


  ros::shutdown();
  return 0;
}

