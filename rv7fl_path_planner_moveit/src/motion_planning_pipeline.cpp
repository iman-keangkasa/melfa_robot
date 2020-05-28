#include <pluginlib/class_loader.h>
#include <ros/ros.h>

//Moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main( int argc, char** argv)
{
	ros::init(argc, argv, "motion_planning_pipeline_node");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle("~");


	// We need two object to start loading the planner with planning pipeline
	// a RobotModel and a PlanningScene
	//To load the robot we will find the robot_description parameter
	//which will be parse by the RobotModelLoader object
	//robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	//robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	robot_model_loader::RobotModelLoaderPtr robot_model_loader(
			new robot_model_loader::RobotModelLoader("robot_description"));
	


	// We will construct the scene based on this robot model
	//planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	planning_scene_monitor::PlanningSceneMonitorPtr psm(
			new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
	
	/* listen for planning scene messages on topic /XXX and apply them to the internal
	 * planning scene */
	psm->startSceneMonitor();

	/*listen to changes of world geometry, collision objects, adn (optionally) octomaps */
	psm->startWorldGeometryMonitor();

	/*listen to joint state updates as well as changes in attached collision objects */
	psm->startStateMonitor();

	/* We can use robot_model_loader to get the robot model 
	 * which contains the robot's kinematics */
	robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();
	
	/* We get the most up to date robot state from the PlanningSceneMonitor
	 * by locking the internal planning scene for reading. This lock 
	 * will ensures that the underlying scene isn't updated while we are
	 * reading it's state. RobotState's are useful for computing the
	 * forward and inverse kinematics of the robot among many other uses 
	*/
	robot_state::RobotStatePtr robot_state(
			new robot_state::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

	/* We create a JointModelGroup to keep track of the current robot pose and planning group.
	 * The joint Model group is useful for dealing with one set of joints at a time such as a left
	 * arm or an end effector*/
	const robot_model::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("arm");

	// We setup the Planningpipeline object
	// The object will use all the parameter in the parameter
	// server loaded in the ompl_planning.yaml in rv7fr_dh_floor
	planning_pipeline::PlanningPipelinePtr planning_pipeline(
			new planning_pipeline::PlanningPipeline(robot_model, node_handle, 
																							"planning_plugin",
																							"request_adapters"));

	//Visualization
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("base");
	visual_tools.deleteAllMarkers();

	
	visual_tools.loadRemoteControl();

	//Rvis markers
	
	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
	text_pose.translation().z() = 1.75;
	visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();

	//ros::Duration(10).sleep();
	
	// We can also use visual_tools to wait for user input
	visual_tools.prompt("Press 'next' to start the demo: Pose goal");
	
	//Pose Goal
	//
	// We now create a motion plan request for the arm of RV7FR 
	// specifying the desired pose of the end-effector as input
	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base";
	pose.pose.position.x = -0.3194830058505;
	pose.pose.position.y = -0.488283479775;
	pose.pose.position.z = 0.542373220529;
	
	pose.pose.orientation.x = 0.999534684091;
	pose.pose.orientation.y = 0.000229501768101;
	pose.pose.orientation.z = 0.0305012350522;
	pose.pose.orientation.w = 0.000192985421839;

	// A tolerance of 0.01 m is specified in position
	// and 0.01 radians in orientation 
	std::vector<double> tolerance_pose(3, 0.01);
	std::vector<double> tolerance_angle(3, 0.01);

	// We will create the request as a constraint using a helper function
	// available in the kinematic_constrains package
	
	req.group_name = "arm";
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("link_effector", 
																																												pose, 
																																												tolerance_pose,
																																												tolerance_angle);
	req.goal_constraints.push_back(pose_goal);

	//Before planning, we will need a Read-Only lock on the planning scene so that it 
	//does not modify the world representation while planning
	{
		planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
		/* Now call the pipeline and check whether planning was successful */
		planning_pipeline->generatePlan(lscene, req, res);
	}

	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_ERROR("Could not compute plan successfully");
		return 0;
	}

	//Visualize the result
	//
	ros::Publisher display_publisher = 
		node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;
	/* Visualize the trajectory */
	ROS_INFO("Visualizing the trajectory");
	moveit_msgs::MotionPlanResponse response;
	res.getMessage(response);

	display_trajectory.trajectory_start = response.trajectory_start;
	display_trajectory.trajectory.push_back(response.trajectory);
	display_publisher.publish(display_trajectory);
	visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
	visual_tools.trigger();

	visual_tools.prompt("Press 'next' in to continue to Joint space goal");

	//Joint Space goal
	/*
	 * First, set the state in the planning scene to the final state of
	 * the last plan*/
	robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
	robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
	robot_state::robotStateToRobotStateMsg(*robot_state, req.start_state);
	
	//Now, we setup a joint space goal
	robot_state::RobotState goal_state(*robot_state);
	std::vector<double> joint_values = {-2.05158474573,-0.299955885374,-0.728439784237,0.0621787989432,1.05722549918,4.7196978940};
	goal_state.setJointGroupPositions(joint_model_group, joint_values);
	moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
	req.goal_constraints.clear();
	req.goal_constraints.push_back(joint_goal);

	//Before planning, we will need a read-only lock on the planning scene so that it does not modify
	//the world representaion while planning
	{
		planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
		/* Now, call the pipeline and check whether planning was successful. */
		planning_pipeline->generatePlan(lscene, req, res);
	}
	/* Check that the planning was successful */
	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_ERROR("Could not compute plan successfully");
		return 0;
	}

	ROS_INFO("Visualizing the trajectory");
	res.getMessage(response);
	display_trajectory.trajectory_start = response.trajectory_start;
	display_trajectory.trajectory.push_back(response.trajectory);
	display_publisher.publish(display_trajectory);
	visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
	visual_tools.trigger();

	/*Wait for user input */
	visual_tools.prompt("Press next to continue using Planning Request Adapter");

	// Using a Planning Request Adapter
	//
	// A planning request adapter allows us to specify a series of operations
	// that should happen either before planning takes place or after the planning
	// has been done on the resultant path
	
	/* First, set the state in the planning scene to the final state of the last plan */
	robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
	robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
	robot_state::robotStateToRobotStateMsg(*robot_state, req.start_state);

	// Now, set one of the joints slightly outside its upper limit
	const robot_model::JointModel* joint_model = joint_model_group->getJointModel("joint3");
	const robot_model::JointModel::Bounds& joint_bounds = joint_model->getVariableBounds();
	std::vector<double> tmp_values(1, 0.0);
	tmp_values[0] = joint_bounds[0].min_position_ - 0.01;
	robot_state->setJointPositions(joint_model, tmp_values);

	req.goal_constraints.clear();
	req.goal_constraints.push_back(pose_goal);

	// Before planning, we will need a read-only lock on the planning scene so that it does not modify
	// the world representation while planning
	{
		planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
		/* Now, call the pipeline and check wheterh planning was successful */
		planning_pipeline->generatePlan(lscene, req, res);
	}
	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_ERROR("Could not compute plan successfully");
		return 0;
	}

	ROS_INFO("Visualizing the trajectory");
	res.getMessage(response);
	display_trajectory.trajectory_start = response.trajectory_start;
	display_trajectory.trajectory.push_back(response.trajectory);
	ROS_INFO("Now I should see three planned trajectories in series");
	display_publisher.publish(display_trajectory);
	visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
	visual_tools.trigger();

	visual_tools.prompt("Press next to conclude the this motion_planning_pipeline");

	ROS_INFO("DONE");
	return 0;
}

	
	

















































	

