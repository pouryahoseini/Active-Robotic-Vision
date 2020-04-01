/*
 * RobotMovement.h
 *
 *  Created on: Mar 8, 2018
 *      Author: pourya
 */

//Guard
#ifndef SOURCECODE_ROBOTMOVEMENT_H_
#define SOURCECODE_ROBOTMOVEMENT_H_

//Headers
#include <iostream>
#include <cmath>
#include <map>
#include <mutex>
#include <atomic>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <visualization_msgs/Marker.h>

#include "GlobalDefinitions.h"

//namespaces
using namespace std;

//Declaring dummy atmic bool variables to be used as default arguments for the below functions
atomic<bool> dummy_isFinished, dummy_isInProgess;

//Declaring global mutexes for locking movement indicators
mutex moveFinished_lock, moveInProgress_lock;

//The class to deal with robot movements
class RobotMovement
{
public:
  //Constructor
  RobotMovement(ros::AsyncSpinner spinner);

  //The function moves left hand by setting joint-space using the member function "moveHandJoints". It tries twice if the first try fails.
  bool moveLeftHand(float shoulder_pan, float shoulder_lift, float upper_arm_roll, float elbow_flex, float forearm_roll, atomic<bool> & isFinished = dummy_isFinished, const bool restBeforeMoving = true);

  //The function to move the hand to the default pose. The default hand is the left. It returns a boolean value to indicate if the movement was successful
  bool moveHandToDefaultPose(atomic<bool> & isInProgess = dummy_isInProgess, bool rightHand = false, const string planningGroupName = "left_arm", const string defaultPoseName = "leftHand_tableTop");

  //Function to perform the preliminary tasks
  void runInitialTasks();

  //The function to move the PR2's parts after all operations are finished
  void releaseTimeActions();

private:
  //The function to move the PR2's hand by setting the joint values. The default hand is the left. It returns a boolean value to indicate if the movement was successful
  bool moveHandJoints(float shoulder_pan, float shoulder_lift, float upper_arm_roll, float elbow_flex, float forearm_roll, const string planningGroupName = "left_arm", const float goalTolerance = 0.05);

  //The function to move the PR2's hand of the robot to a 3D location with a predefined orientation. The default hand is the left. It returns a boolean value to indicate if the movement was successful
  bool moveHandToTarget(float x, float y, float z, float roll, float pitch, float yaw, const string planningGroupName = "left_arm");

  //The function to move the PR2's head by setting the joint values. It returns a boolean value to indicate if the movement was successful
  bool moveHeadJoints(float head_pan, float head_tilt);

  //The function to move the PR2's torso by setting the joint values. It returns a boolean value to indicate if the movement was successful
  bool moveTorsoJoint(float torso_lift);

  //The functions to move to the default poses. They return boolean values to indicate if the movement was successful
  bool moveHeadToDefaultPose(const string defaultPoseName = "head_tableTop");
  bool moveTorsoToDefaultPose(const string defaultPoseName = "torso_tableTop");

  //The function to set collision objects
  //It sets the collision objects for the left and right hands of the robot only
  void defineCollisionObjects();

  const string leftHand_PlanningGroupName = "left_arm", rightHand_PlanningGroupName = "right_arm", head_PlanningGroupName = "head", torso_PlanningGroupName = "torso";
  const string leftHand_defaultPoseName = "leftHand_tableTop", rightHand_defaultPoseName = "rightHand_tableTop", head_defaultPoseName = "head_tableTop",torso_defaultPoseName = "torso_tableTop";
  const string leftHand_tuck = "tuck_left_arm", rightHand_tuck = "tuck_right_arm", head_moveBack = "head_release", torso_moveBack = "torso_release";
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  geometry_msgs::Pose targetPose;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;
  moveit::core::RobotStatePtr current_state;
  std::vector<double> hand_jointGroupPositions, head_jointGroupPositions, torso_jointGroupPositions;
  //tf::Quaternion poseQuanternion;
  moveit_msgs::JointConstraint wristFlex_jointConstraint, wristRoll_jointConstraint;
  moveit_msgs::Constraints constraints;
  map<string, double> hand_defaultPose, head_defaultPose, torso_defaultPose;
  moveit_msgs::CollisionObject collision_object;
  shape_msgs::SolidPrimitive objectAvoidanceBox;
  geometry_msgs::Pose box_pose;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  mutex publicFunctions_Lock, jointMoverFunction_mutex;
};

#endif /* SOURCECODE_ROBOTMOVEMENT_H_ */
