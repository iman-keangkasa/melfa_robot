cmake_minimum_required(VERSION 3.0.2)
project(rv7fr_planners_interface)

find_package(Eigen3 REQUIRED)
if (NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost REQUIRED system filesystem date_time thread)

find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  rospy
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rv7fl_path_planner_moveit
  CATKIN_DEPENDS 
  geometric_shapes 
  interactive_markers 
  moveit_core 
  moveit_msgs 
  moveit_ros_perception 
  moveit_ros_planning_interface 
  moveit_visual_tools 
  pcl_conversions 
  pcl_ros pluginlib 
  roscpp 
  rospy 
  tf2_geometry_msgs tf2_eigen tf2_ros
  DEPENDS EIGEN3
)

include_directories( SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARIES_DIRS})

set(POSE_MARKER_UTILS rv7_pose_marker_util)
add_library( ${POSE_MARKER_UTILS} src/pose_marker_class.cpp )

#add_executable(move_group_interface src/move_group_interface.cpp)
#target_link_libraries(move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
target_link_libraries(obstacle_avoidance ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(obstacle src/obstacle.cpp)
target_link_libraries(obstacle ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(simple_move_group src/simple_move_group.cpp)
target_link_libraries(simple_move_group ${POSE_MARKER_UTILS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testing_msgs src/testing_msgs.cpp)
target_link_libraries(testing_msgs ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_state_model src/robot_state_model.cpp)
target_link_libraries(robot_state_model ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pose_marker src/pose_marker.cpp)
target_link_libraries(pose_marker ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_class src/test_class.cpp)
target_link_libraries(test_class ${POSE_MARKER_UTILS} ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
