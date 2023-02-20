/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra*/

#include <iostream>

// ROS
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// ED
#include <ed_msgs/SimpleQuery.h>
#include <ed_msgs/EntityInfo.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group, const ed_msgs::EntityInfo& entity)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  
  // Get entity info
  geometry_msgs::Pose entity_pose = entity.pose;

  double palm_offset = 0.1;

  // Setting grasp poses
  // Step 1, near side of box
  double dx = 0.01; // m

  
  for (int i = 0; i < 19; i++)
  {
    moveit_msgs::Grasp grasp;
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
    // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
    // transform from `"panda_link8"` to the palm of the end effector.
    grasp.grasp_pose.header.frame_id = "map";
    tf2::Quaternion orientation;
    orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
    grasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasp.grasp_pose.pose.position.x = entity.pose.position.x - 0.065 - palm_offset;
    grasp.grasp_pose.pose.position.y = entity.pose.position.y;
    grasp.grasp_pose.pose.position.z = entity.pose.position.z + i*dx;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    // Defined with respect to frame_id 
    grasp.pre_grasp_approach.direction.header.frame_id = "panda_link8";
    // Direction is set as positive x axis
    grasp.pre_grasp_approach.direction.vector.z = 1.0;
    grasp.pre_grasp_approach.min_distance = 0.095;
    grasp.pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++89
    // Defined with respect to frame_id 
    grasp.post_grasp_retreat.direction.header.frame_id = "panda_link0";
    // Direction is set as positive z axis
    grasp.post_grasp_retreat.direction.vector.z = 1.0;
    grasp.post_grasp_retreat.min_distance = 0.1;
    grasp.post_grasp_retreat.desired_distance = 0.25;
  
    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasp.pre_grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick2
    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasp.grasp_posture);
    // END_SUB_TUTORIAL
    grasps.push_back(grasp);
  }

  // Step 2, top of box
  for (int i = 0; i < 13; i++)
  {
    moveit_msgs::Grasp grasp;
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
    // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
    // transform from `"panda_link8"` to the palm of the end effector.
    grasp.grasp_pose.header.frame_id = "map";
    tf2::Quaternion orientation;
    orientation.setRPY(tau / 2, 0.0, -tau / 8);
    grasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasp.grasp_pose.pose.position.x = entity.pose.position.x - 0.065 + i*dx;
    grasp.grasp_pose.pose.position.y = entity.pose.position.y;
    grasp.grasp_pose.pose.position.z = entity.pose.position.z + 0.195 + palm_offset;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    // Defined with respect to frame_id
    grasp.pre_grasp_approach.direction.header.frame_id = "panda_link8";
    // Direction is set as positive x axis
    grasp.pre_grasp_approach.direction.vector.z = 1.0;
    grasp.pre_grasp_approach.min_distance = 0.095;
    grasp.pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++89
    // Defined with respect to frame_id
    grasp.post_grasp_retreat.direction.header.frame_id = "panda_link0";
    // Direction is set as positive z axis
    grasp.post_grasp_retreat.direction.vector.z = 1.0;
    grasp.post_grasp_retreat.min_distance = 0.1;
    grasp.post_grasp_retreat.desired_distance = 0.25;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasp.pre_grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick2
    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasp.grasp_posture);
    // END_SUB_TUTORIAL
    grasps.push_back(grasp);
  }

  // step 3 back of box
  for (int i = 0; i < 19; i++)
  {
    moveit_msgs::Grasp grasp;
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
    // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
    // transform from `"panda_link8"` to the palm of the end effector.
    grasp.grasp_pose.header.frame_id = "map";
    tf2::Quaternion orientation;
    orientation.setRPY(-tau / 4, -tau / 8, tau / 4);
    grasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasp.grasp_pose.pose.position.x = entity.pose.position.x + 0.065 + palm_offset;
    grasp.grasp_pose.pose.position.y = entity.pose.position.y;
    grasp.grasp_pose.pose.position.z = entity.pose.position.z + i*dx;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    // Defined with respect to frame_id
    grasp.pre_grasp_approach.direction.header.frame_id = "panda_link8";
    // Direction is set as positive x axis
    grasp.pre_grasp_approach.direction.vector.z = 1.0;
    grasp.pre_grasp_approach.min_distance = 0.095;
    grasp.pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++89
    // Defined with respect to frame_id
    grasp.post_grasp_retreat.direction.header.frame_id = "panda_link0";
    // Direction is set as positive z axis
    grasp.post_grasp_retreat.direction.vector.z = 1.0;
    grasp.post_grasp_retreat.min_distance = 0.1;
    grasp.post_grasp_retreat.desired_distance = 0.25;
  
    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasp.pre_grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick2
    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasp.grasp_posture);
    // END_SUB_TUTORIAL
    grasps.push_back(grasp);
  }

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  move_group.setSupportSurfaceName("table");
  // Call pick to pick up the object using the grasps given
  move_group.pick("rice1", grasps);
  // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface& group, ed_msgs::EntityInfo& entity)
{
  // Get entity info
  geometry_msgs::Pose entity_pose = entity.pose;

  double palm_offset = 0.1;

  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in verbose mode." This is a known issue. |br|
  // |br|
  // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
  // a single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "map";
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, tau / 4, 0.0);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* For place location, we set the value to the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = entity.pose.position.x - 0.0975;
  place_location[0].place_pose.pose.position.y = entity.pose.position.y;
  place_location[0].place_pose.pose.position.z = entity.pose.position.z + 0.1 + palm_offset;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table");
  // Call place to place the object using the place locations given.
  group.place("rice1", place_location);
  // END_SUB_TUTORIAL
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(45.0);

  // get collision environment from ED
  ros::ServiceClient ed_scene_client = nh.serviceClient<std_srvs::Trigger>("ed/moveit_scene");
  std_srvs::Trigger req;
  ed_scene_client.call(req);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  // get ED entity to grab
  ros::ServiceClient ed_client = nh.serviceClient<ed_msgs::SimpleQuery>("ed/simple_query");
  ed_msgs::SimpleQuery srv;
  srv.request.id = "rice1";
  srv.request.radius = 100;
  if (!ed_client.call(srv))
  {
    ROS_ERROR("cannot query entity rice1");
    return 1;
  }
  ed_msgs::EntityInfo entity = srv.response.entities[0];

  srv.request.id = "cardboard_box";
  if (!ed_client.call(srv))
  {
    ROS_ERROR("cannot query entity cardboard_box");
    return 1;
  }
  ed_msgs::EntityInfo place_entity = srv.response.entities[0];

  ROS_INFO("starting pick");
  pick(group, entity);
  ROS_INFO("finished pick");

  ros::WallDuration(1.0).sleep();

  std::cout << "press Enter to continue" << std::endl;
  std::cin.get();

  ROS_INFO("starting place");
  place(group, place_entity);
  ROS_INFO("finished place");

  ros::waitForShutdown();
  return 0;
}
