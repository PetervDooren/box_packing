cmake_minimum_required(VERSION 3.0.2)
project(moveit_box_packing)


find_package(catkin REQUIRED
    COMPONENTS
    ed
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
    LIBRARIES
    INCLUDE_DIRS
    CATKIN_DEPENDS
      moveit_core
      moveit_visual_tools
      moveit_ros_planning_interface
      interactive_markers
      tf2_geometry_msgs
    DEPENDS
      EIGEN3
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(pick_place_node src/pick_place.cpp)
target_link_libraries(pick_place_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

add_executable(motion_planning_test src/motion_planning_api_tutorial.cpp)
target_link_libraries(motion_planning_test
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
