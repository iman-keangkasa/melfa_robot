#include <ros/ros.h>
#include "rv7fl_path_planner_moveit/interactive_robot.h"
#include "rv7fl_path_planner_moveit/pose_string.h"

//Moveit
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <moveit/collision_detection/collision_tools.h>

planning_scene::PlanningScene* g_planning_scene = 0;
shapes::ShapePtr g_world_cube_shape;
ros::Publisher* g_marker_array_publisher = 0;
visualization_msgs::MarkerArray g_collision_points;

void help()
{
	ROS_INFO("#####################################################");
  ROS_INFO("RVIZ SETUP");
  ROS_INFO("----------");
  ROS_INFO("  Global options:");
  ROS_INFO("    FixedFrame = /base");
  ROS_INFO("  Add a RobotState display:");
  ROS_INFO("    RobotDescription = robot_description");
  ROS_INFO("    RobotStateTopic  = interactive_robot_state");
  ROS_INFO("  Add a Marker display:");
  ROS_INFO("    MarkerTopic = interactive_robot_markers");
  ROS_INFO("  Add an InteractiveMarker display:");
  ROS_INFO("    UpdateTopic = interactive_robot_imarkers/update");
  ROS_INFO("  Add a MarkerArray display:");
  ROS_INFO("    MarkerTopic = interactive_robot_marray");
  ROS_INFO("#####################################################");
}

void publishMarkers(visualization_msgs::MarkerArray& markers)
{
	//delete old markers
	if (g_collision_points.markers.size())
	{
		for(int i = 0; i < g_collision_points.markers.size(); i++)
			g_collision_points.markers[i].action = visualization_msgs::Marker::DELETE;
		
		g_marker_array_publisher->publish(g_collision_points);
	}

	// move new markers into g_collision_points
	std::swap(g_collision_points.markers, markers.markers);

	// draw new markers into g_collision_points
	if (g_collision_points.markers.size())
		g_marker_array_publisher->publish(g_collision_points);
}

void computeCollisionContactPoints(InteractiveRobot& robot)
{
	//move the world geometry in the collision world
	Eigen::Isometry3d world_cube_pose;
	double world_cube_size;
	robot.getWorldGeometry(world_cube_pose, world_cube_size);
	g_planning_scene->getWorldNonConst()->moveShapeInObject("world_cube", g_world_cube_shape, world_cube_pose);
	
	// BEGIN_SUB_TUTORIAL computeColisionContactPoints
	//
	// Collision Requests
	//
	// We will create a collision request for the rv7 robot
	collision_detection::CollisionRequest c_req;
	collision_detection::CollisionResult c_res;
	c_req.group_name = robot.getGroupName();
	c_req.contacts = true;
	c_req.max_contacts = 100;
	c_req.max_contacts_per_pair = 5;
	c_req.verbose = false;

	// Checking for collisions
	// 
	// We check for collisions between robot and itself or the world
	g_planning_scene->checkCollision(c_req, c_res, *robot.robotState());

	// Displaying Collision Contact Points
	//
	// If there are collision, we get the contact points and display 
	// them as markers.
	// **getCollisionMarkersFromContacts()** is a helper function that adds
	// the collision contact points into a MarkerArray message. If you want to 
	// use the contact points for something other than displaying them you can
	// iterate through **c_res.contacts**
  // Look at the implementation of getCOllisionMarkersFromContacts in collision_tools.cpp
	
	if (c_res.collision)
	{
		ROS_INFO("COLLIDING contact_point_count=%d", (int)c_res.contact_count);
		if (c_res.contact_count >0)
		{
			std_msgs::ColorRGBA color;
			color.r = 1.0;
			color.g = 0.0;
			color.b = 1.0;
			color.a = 0.5;
			visualization_msgs::MarkerArray markers;

			/* Get the contact points and display them as marker */
			collision_detection::getCollisionMarkersFromContacts(markers, "base", c_res.contacts, color,
																													 ros::Duration(), //remain until deleted
																													 0.01); // radius
			publishMarkers(markers);
		}
	}
	// END_SUB_TUTORIAL
	else
	{
		ROS_INFO("Not colliding");
		// delete the olf collision point markers
		visualization_msgs::MarkerArray empty_marker_array;
		publishMarkers(empty_marker_array);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "visualizing_collision_node");
	ros::NodeHandle nh;

	// Initializing the Planning scene and Markers
	//
	// We will use 'InteractiveRobot <interactivity/src/interactive_robot.cpp>
	// object as a wrapper that combines a robot model with the cube and an interactive marker. We also 
	// create a planning scene 'PlanningScene' for collision_checking.
	
	InteractiveRobot robot;

	g_planning_scene = new planning_scene::PlanningScene(robot.robotModel());

	// Adding geometry to the PlanningScene
	Eigen::Isometry3d world_cube_pose;
	double world_cube_size;
	robot.getWorldGeometry(world_cube_pose, world_cube_size);
	g_world_cube_shape.reset( new shapes::Box(world_cube_size, world_cube_size, world_cube_size));
	g_planning_scene->getWorldNonConst()->addToObject("world_cube", g_world_cube_shape, world_cube_pose);

	// CALL_SUB_TUTORIAL computeCollisionContactPoints
	// END_TUTORIAL
	
	// Create a marker array publisher for publishing contact points
	g_marker_array_publisher = 
		new ros::Publisher(nh.advertise<visualization_msgs::MarkerArray>("interactive_robot_marray", 100));

	robot.setUserCallback(computeCollisionContactPoints);
	help();
	ros::spin();

	delete g_planning_scene;
	delete g_marker_array_publisher;

	ros::shutdown();
	return 0;
}



























































			
