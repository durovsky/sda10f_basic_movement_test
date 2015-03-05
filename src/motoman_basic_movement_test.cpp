/*********************************************************************************************//**
* @file motoman_basic_movement_test.cpp
*
* Simple node for testing move group action with motoman SDA10F robot
* Node works with basic motoman_sda10f_moveit_config, no predeffined robot poses are required 
* 
* Copyright (c)
* Frantisek Durovsky
* Department of Robotics
* Technical University Kosice
* February 2015
*   
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* *********************************************************************************************/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

int
main(int argc, char *argv[])
{
  ros::init(argc, argv, "motoman_basic_movement_test");
  ros::NodeHandle nh;
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  moveit::planning_interface::MoveGroup group_arm_left("arm_left");
  moveit::planning_interface::MoveGroup group_arm_right("arm_right");
  moveit::planning_interface::MoveGroup torso("torso");
  
  //===========================================
  //Read and set parameters from launch file
  //===========================================
  double planningTime;
  double orientationTolerance;
  double positionTolerance;

  //Read parameters from launch file
  nh.getParam("planning_time", planningTime);
  nh.getParam("orientation_tolerance", orientationTolerance);
  nh.getParam("position_tolerance", positionTolerance);

  //Set moveit_planning_interface parameters
  group_arm_left.setPlanningTime(planningTime);
  group_arm_left.setGoalOrientationTolerance(orientationTolerance);
  group_arm_left.setGoalPositionTolerance(positionTolerance);
  
  group_arm_right.setPlanningTime(planningTime);
  group_arm_right.setGoalOrientationTolerance(orientationTolerance);
  group_arm_right.setGoalPositionTolerance(positionTolerance);

  torso.setPlanningTime(planningTime);
  torso.setGoalOrientationTolerance(orientationTolerance);
  torso.setGoalPositionTolerance(positionTolerance);
  
  //Wait until everything starts up
  ros::Duration(10).sleep();
  
  //==========================================================
  //Robot Initial Manipulation
  //==========================================================
  moveit::planning_interface::MoveGroup::Plan arm_left_plan;
  moveit::planning_interface::MoveGroup::Plan arm_right_plan;
  moveit::planning_interface::MoveGroup::Plan torso_plan;
  
  std::cout << "Press Enter to start robot movemement" << std::endl;
  std::cin.get();
  
  //Small initial torso movement to power up servo controller
  std::vector<double> initial_torso_joint_values;
  torso.getCurrentState()->copyJointGroupPositions(torso.getCurrentState()->getRobotModel()->getJointModelGroup(torso.getName()), initial_torso_joint_values);
  for(size_t i = 0; i < initial_torso_joint_values.size(); i++)
    initial_torso_joint_values[i] -= 0.01;
  torso.setJointValueTarget(initial_torso_joint_values);
  torso.plan(torso_plan);  
  torso.execute(torso_plan);
  ros::Duration(3).sleep();
  
  //==========================================================
  //Move to zero position
  //==========================================================
  
  //Planning to a joint-space goal - zero torso position
  std::vector<double> torso_joint_values;
  torso.getCurrentState()->copyJointGroupPositions(torso.getCurrentState()->getRobotModel()->getJointModelGroup(torso.getName()), torso_joint_values);
  for(size_t i = 0; i < torso_joint_values.size(); i++)
    torso_joint_values[i] = 0;
  torso.setJointValueTarget(torso_joint_values);
  torso.plan(torso_plan);
  torso.execute(torso_plan);
  ros::Duration(3).sleep();
  
  //Planning to a joint-space goal - zero arm_left positions
  std::vector<double> arm_left_joint_values;
  group_arm_left.getCurrentState()->copyJointGroupPositions(group_arm_left.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm_left.getName()), arm_left_joint_values);
  for(size_t i = 0; i < arm_left_joint_values.size(); i++)
    arm_left_joint_values[i] = 0;
  group_arm_left.setJointValueTarget(arm_left_joint_values);
  group_arm_left.plan(arm_left_plan);
  group_arm_left.execute(arm_left_plan);
  ros::Duration(3).sleep();
  
  //Planning to a joint-space goal - zero arm_right positions
  std::vector<double> arm_right_joint_values;
  group_arm_right.getCurrentState()->copyJointGroupPositions(group_arm_right.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm_right.getName()), arm_right_joint_values);
  for(size_t i = 0; i < arm_right_joint_values.size(); i++)
    arm_right_joint_values[i] = 0;
  group_arm_right.setJointValueTarget(arm_right_joint_values);
  group_arm_right.plan(arm_right_plan);
  group_arm_right.execute(arm_right_plan);
  ros::Duration(3).sleep();
    
  //==========================================================
  //Move to grasp position
  //==========================================================
    
  //Planning to a joint-space goal - arm_left grasp position
  group_arm_left.getCurrentState()->copyJointGroupPositions(group_arm_left.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm_left.getName()), arm_left_joint_values);
  //Grasp position joint values
  arm_left_joint_values[0] = -0.707;
  arm_left_joint_values[1] = 1;
  arm_left_joint_values[2] = 0.707;
  arm_left_joint_values[3] = -1.57;
  arm_left_joint_values[4] = -0.9;
  arm_left_joint_values[5] = -1.2168;
  arm_left_joint_values[6] = 0;
    
  group_arm_left.setJointValueTarget(arm_left_joint_values);
  group_arm_left.plan(arm_left_plan);
  group_arm_left.execute(arm_left_plan);
  ros::Duration(3).sleep();
  
  //Planning to a joint-space goal - arm_right grasp position
  group_arm_right.getCurrentState()->copyJointGroupPositions(group_arm_right.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm_right.getName()), arm_right_joint_values);
  //Grasp position joint values
  arm_right_joint_values[0] = -0.707;
  arm_right_joint_values[1] = 1;
  arm_right_joint_values[2] = 0.707;
  arm_right_joint_values[3] = -1.57;
  arm_right_joint_values[4] = -0.9;
  arm_right_joint_values[5] = -1.2168;
  arm_right_joint_values[6] = 0;
  
  group_arm_right.setJointValueTarget(arm_right_joint_values);
  group_arm_right.plan(arm_right_plan);
  group_arm_right.execute(arm_right_plan);
  ros::Duration(3).sleep();
  
  //Planning to a joint-space goal - torso 90 degrees left position
  torso.getCurrentState()->copyJointGroupPositions(torso.getCurrentState()->getRobotModel()->getJointModelGroup(torso.getName()), torso_joint_values);
  for(size_t i = 0; i < torso_joint_values.size(); i++)
    torso_joint_values[i] = 1.57;
  torso.setJointValueTarget(torso_joint_values);
  torso.plan(torso_plan);
  torso.execute(torso_plan);
  ros::Duration(3).sleep();
 
  while(ros::ok())
  { 
    //=================================================================
    //Cartesian arm_left movement
    //=================================================================
    geometry_msgs::PoseStamped arm_left_actual_pose;
    geometry_msgs::Pose arm_left_waypoint_01,
			arm_left_waypoint_02,
			arm_left_waypoint_03,
			arm_left_waypoint_04;
			
    arm_left_actual_pose = group_arm_left.getCurrentPose("arm_left_link_7_t");
    
    //Copy current effector position and orientation
    arm_left_waypoint_01.position = arm_left_actual_pose.pose.position;
    arm_left_waypoint_02.position = arm_left_actual_pose.pose.position;  
    arm_left_waypoint_03.position = arm_left_actual_pose.pose.position;
    arm_left_waypoint_04.position = arm_left_actual_pose.pose.position;
    
    arm_left_waypoint_01.orientation = arm_left_actual_pose.pose.orientation;
    arm_left_waypoint_02.orientation = arm_left_actual_pose.pose.orientation;
    arm_left_waypoint_03.orientation = arm_left_actual_pose.pose.orientation;
    arm_left_waypoint_04.orientation = arm_left_actual_pose.pose.orientation;
    
    //Change position values to differentiate waypoints
    arm_left_waypoint_01.position.z -= 0.2;			  //current  < -    w03
    arm_left_waypoint_02.position.z -= 0.2;                       //    |            |
    arm_left_waypoint_02.position.x -= 0.2;                       //    |            |
    arm_left_waypoint_03.position.x -= 0.2;                       //   w01   - >    w02
    
    std::vector<geometry_msgs::Pose> arm_left_cartesian_waypoints;
    moveit_msgs::RobotTrajectory arm_left_trajectory;
    
    arm_left_cartesian_waypoints.push_back(arm_left_waypoint_01);
    arm_left_cartesian_waypoints.push_back(arm_left_waypoint_02);
    arm_left_cartesian_waypoints.push_back(arm_left_waypoint_03);
    arm_left_cartesian_waypoints.push_back(arm_left_waypoint_04);
    
    double arm_left_fraction = group_arm_left.computeCartesianPath(arm_left_cartesian_waypoints, 0.02, 0, arm_left_trajectory, false);
    ROS_INFO("Cartesian path) (%.2f%% acheived)", arm_left_fraction * 100.0);
    
    //If computation was successfull move the arm
    if(arm_left_fraction == 1)
    {
      //Adding velocities to cartesian trajectory
      robot_trajectory::RobotTrajectory arm_left_rt_01(group_arm_left.getCurrentState()->getRobotModel(),"arm_left");
      arm_left_rt_01.setRobotTrajectoryMsg(*group_arm_left.getCurrentState(), arm_left_trajectory);
      trajectory_processing::IterativeParabolicTimeParameterization arm_left_iptp;
      
      //Compute Time Stamps
      const double max_velocity_scaling_factor = 0.01;
      bool arm_left_success = arm_left_iptp.computeTimeStamps(arm_left_rt_01,max_velocity_scaling_factor);
      ROS_INFO("Computed time stamps: %s", arm_left_success ? "SUCCEDED" : "FAILED");

      //Get robot trajectory from RobotTrajectory
      arm_left_rt_01.getRobotTrajectoryMsg(arm_left_trajectory);
      arm_left_plan.trajectory_ = arm_left_trajectory;
      //ROS_INFO_STREAM("Trajectory" << arm_left_trajectory);
      group_arm_left.execute(arm_left_plan);    
      ros::Duration(3).sleep();
      
    }
    
    //=================================================================
    //Cartesian arm_right movement
    //=================================================================
    geometry_msgs::PoseStamped arm_right_actual_pose;
    geometry_msgs::Pose arm_right_waypoint_01,
			arm_right_waypoint_02,
			arm_right_waypoint_03,
			arm_right_waypoint_04;
			
    arm_right_actual_pose = group_arm_right.getCurrentPose("arm_right_link_7_t");
    
    //Copy current effector position and orientation
    arm_right_waypoint_01.position = arm_right_actual_pose.pose.position;
    arm_right_waypoint_02.position = arm_right_actual_pose.pose.position;  
    arm_right_waypoint_03.position = arm_right_actual_pose.pose.position;
    arm_right_waypoint_04.position = arm_right_actual_pose.pose.position;
    
    arm_right_waypoint_01.orientation = arm_right_actual_pose.pose.orientation;
    arm_right_waypoint_02.orientation = arm_right_actual_pose.pose.orientation;
    arm_right_waypoint_03.orientation = arm_right_actual_pose.pose.orientation;
    arm_right_waypoint_04.orientation = arm_right_actual_pose.pose.orientation;
    
    //Change position values to differentiate waypoints
    arm_right_waypoint_01.position.z -= 0.2;			   //    w03  ->    current
    arm_right_waypoint_02.position.z -= 0.2;                       //    |            |
    arm_right_waypoint_02.position.x += 0.2;                       //    |            |
    arm_right_waypoint_03.position.x += 0.2;                       //   w02   < -    w01
    
    std::vector<geometry_msgs::Pose> arm_right_cartesian_waypoints;
    moveit_msgs::RobotTrajectory arm_right_trajectory;
    
    arm_right_cartesian_waypoints.push_back(arm_right_waypoint_01);
    arm_right_cartesian_waypoints.push_back(arm_right_waypoint_02);
    arm_right_cartesian_waypoints.push_back(arm_right_waypoint_03);
    arm_right_cartesian_waypoints.push_back(arm_right_waypoint_04);
    
    double arm_right_fraction = group_arm_right.computeCartesianPath(arm_right_cartesian_waypoints, 0.02, 0, arm_right_trajectory, false);
    ROS_INFO("Cartesian path) (%.2f%% acheived)", arm_right_fraction * 100.0);
    
    //If computation was successfull move the arm
    if(arm_right_fraction == 1)
    {
      //Adding velocities to cartesian trajectory
      robot_trajectory::RobotTrajectory arm_right_rt_01(group_arm_right.getCurrentState()->getRobotModel(),"arm_right");
      arm_right_rt_01.setRobotTrajectoryMsg(*group_arm_right.getCurrentState(), arm_right_trajectory);
      trajectory_processing::IterativeParabolicTimeParameterization arm_right_iptp;
      
      //Compute Time Stamps
      const double max_velocity_scaling_factor = 1.0;
      bool arm_right_success = arm_right_iptp.computeTimeStamps(arm_right_rt_01,max_velocity_scaling_factor);
      ROS_INFO("Computed time stamps: %s", arm_right_success ? "SUCCEDED" : "FAILED");

      //Get robot trajectory from RobotTrajectory
      arm_right_rt_01.getRobotTrajectoryMsg(arm_right_trajectory);
      arm_right_plan.trajectory_ = arm_right_trajectory;
      //ROS_INFO_STREAM("Trajectory" << arm_right_trajectory);
      group_arm_right.execute(arm_right_plan); 
      ros::Duration(3).sleep();
      
    }
  }
    
  return(EXIT_SUCCESS);
}




























