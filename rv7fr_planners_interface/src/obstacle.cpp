//#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }



  //Obstacle setting
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "table";
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
  
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(collision_object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

  ros::Duration(5).sleep();
  //attempting to move the object 
 
  int tempo(0);
  while(ros::ok())
  {
    
    moveit_msgs::CollisionObject move_object;
    move_object.header.frame_id = "table";
    move_object.id = "cylinder";
 
    geometry_msgs::Pose cylinder_move;
    cylinder_move.orientation.w = 1.0;
    cylinder_move.position.x = 1.55+sin(0.06*tempo/100.0f*2*M_PI);
    cylinder_move.position.y = 0;
    cylinder_move.position.z = 0.7;
    move_object.primitive_poses.push_back(cylinder_move);
    move_object.operation = collision_object.MOVE;
    //planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(move_object);
    planning_scene_diff_publisher.publish(planning_scene);
    
    ++tempo;

    ros::Duration(0.006).sleep();
  }
  
  
   
  ros::shutdown();

  return 0;
}





