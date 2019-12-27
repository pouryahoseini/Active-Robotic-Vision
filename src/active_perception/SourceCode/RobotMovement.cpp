/*
 * RobotMovement.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: pourya
 */

//Header
#include "RobotMovement.h"

/*******************************/
//Constructor
RobotMovement::RobotMovement(ros::AsyncSpinner spinner)
{
  //Start a ROS asynchronous spinner
  spinner.start();

  /* Constraints for the target-space planning */
  //Constructing the wrist flex joint constraint
  wristFlex_jointConstraint.joint_name = "l_wrist_flex_joint";
  wristFlex_jointConstraint.position = -0.1;
  wristFlex_jointConstraint.tolerance_above = 0.1;
  wristFlex_jointConstraint.tolerance_below = 0.1;
  wristFlex_jointConstraint.weight = 1.0;
  constraints.joint_constraints.push_back(wristFlex_jointConstraint);

  //Constructing the wrist roll joint constraint
  wristRoll_jointConstraint.joint_name = "l_wrist_roll_joint";
  wristRoll_jointConstraint.position = 0;
  wristRoll_jointConstraint.tolerance_above = 0.1;
  wristRoll_jointConstraint.tolerance_below = 0.1;
  wristRoll_jointConstraint.weight = 1.0;
  constraints.joint_constraints.push_back(wristRoll_jointConstraint);
}

/*******************************/
//Function to perform the preliminary tasks
void RobotMovement::runInitialTasks()
{
  //Do operations only if simulation is enabled
  if(SIMULATION_WORLD_ENABLE == true)
    {
      //Move the PR2's left arm, head, and torso to the default poses
      moveTorsoToDefaultPose();
      moveHeadToDefaultPose();
      moveHandToDefaultPose(std::ref(dummy_isInProgess));
      moveHandToDefaultPose(std::ref(dummy_isInProgess), true, rightHand_PlanningGroupName, rightHand_defaultPoseName);
    }

  //Define the collision objects
  defineCollisionObjects();

  return;
}

/*******************************/
//The function to move the PR2's parts after all operations are finished
void RobotMovement::releaseTimeActions()
{
  //Lock the public functions mutex
  publicFunctions_Lock.lock();

  //Move the PR2's parts to their predefined state before releasing
  moveHandToDefaultPose(std::ref(dummy_isInProgess), false, leftHand_PlanningGroupName, leftHand_tuck);
  moveHandToDefaultPose(std::ref(dummy_isInProgess), true, rightHand_PlanningGroupName, rightHand_tuck);
  moveHeadToDefaultPose(head_moveBack);
  if(! moveTorsoToDefaultPose(torso_moveBack)) //Retry torso movement if it is failed initially
    moveTorsoToDefaultPose(torso_moveBack);

  //Unlock the public functions mutex
  publicFunctions_Lock.unlock();

  return;
}

/*******************************/
//The function moves left hand by setting joint-space using the member function "moveHandJoints". It tries twice if the first try fails.
bool RobotMovement::moveLeftHand(float shoulder_pan, float shoulder_lift, float upper_arm_roll, float elbow_flex, float forearm_roll, atomic<bool> & isFinished, const bool restBeforeMoving)
{cout<<endl<<"Moving hand started"<<endl;
  //Lock the public functions mutex
  publicFunctions_Lock.lock();

  //Indicate that the operation is running
  moveFinished_lock.lock();
  isFinished = false;

  //Rest before moving, if enabled
  //This is used to prevent back to back movements
  if(restBeforeMoving)
    sleep(MOVE_FORWARD_REST_TIME);

  //Declare a local return value
  bool success;

  if( (success = moveHandJoints(shoulder_pan, shoulder_lift, upper_arm_roll, elbow_flex, forearm_roll)) )
    success = moveHandJoints(shoulder_pan, shoulder_lift, upper_arm_roll, elbow_flex, forearm_roll, "left_arm", 0.2);

  //Indicate that the operation is finished, successful or unsuccessful
  isFinished = true;
  moveFinished_lock.unlock();

  //Unlock the public functions mutex
  publicFunctions_Lock.unlock();
  cout<<endl<<"Moving hand finished"<<endl;
  //Return if the move operation was successful
  return success;
}

/*******************************/
//The function to move the PR2's hand by setting the joint values. The default hand is the left. It returns a boolean value to indicate if the movement was successful
bool RobotMovement::moveHandJoints(float shoulder_pan, float shoulder_lift, float upper_arm_roll, float elbow_flex, float forearm_roll, const string planningGroupName, const float goalTolerance)
{
  //Lock this function
  jointMoverFunction_mutex.lock();

  //Creating the move groups
  moveit::planning_interface::MoveGroupInterface hand_group(planningGroupName);
  hand_group.setPlannerId("RRTConnectkConfigDefault");

  //Get the current state of the left hand
  current_state = hand_group.getCurrentState();

  //Get the current state of the joint values
  const robot_state::JointModelGroup *joint_model_group = hand_group.getCurrentState()->getJointModelGroup(planningGroupName);
  current_state->copyJointGroupPositions(joint_model_group, hand_jointGroupPositions);

  //Set joint values
  hand_jointGroupPositions[0] = shoulder_pan;
  hand_jointGroupPositions[1] = shoulder_lift;
  hand_jointGroupPositions[2] = upper_arm_roll;
  hand_jointGroupPositions[3] = elbow_flex;
  hand_jointGroupPositions[4] = forearm_roll;
  hand_jointGroupPositions[5] = 0;
  hand_jointGroupPositions[6] = 0;

  //Apply joint values
  hand_group.setJointValueTarget(hand_jointGroupPositions);

  //Set goal tolerance
  hand_group.setGoalJointTolerance(goalTolerance);

  //Plan the move
  success = (hand_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //Return false if planning was unsuccessful
  if(success == false)
    {
      jointMoverFunction_mutex.unlock();
      return false;
    }

  //Move the arm
  success = (hand_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //Unlock the function
  jointMoverFunction_mutex.unlock();

  return success;
}

/*******************************/
//The function to move the PR2's hand of the robot to a 3D location with a predefined orientation. The default hand is the left. It returns a boolean value to indicate if the movement was successful
bool RobotMovement::moveHandToTarget(float x, float y, float z, float roll, float pitch, float yaw, const string planningGroupName)
{
  //Creating the move groups
  moveit::planning_interface::MoveGroupInterface hand_group(planningGroupName);

  //Applying the wrist joint constraints
  hand_group.setPathConstraints(constraints);

  //Set goal tolerances
  hand_group.setGoalOrientationTolerance(0.1);
  hand_group.setGoalPositionTolerance(0.1);

  //Set the target into the planning group
  hand_group.setRPYTarget(roll, pitch, yaw);
  hand_group.setPositionTarget(x, y, z);
  //leftHand_group.setRandomTarget();

  //Plan the move
  success = (hand_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //Return false if planning was unsuccessful
  if(success == false)
    return false;

  //Move the left arm
  success = (hand_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  return success;
}

/*******************************/
//The function to move the PR2's head by setting the joint values. It returns a boolean value to indicate if the movement was successful
bool RobotMovement::moveHeadJoints(float head_pan, float head_tilt)
{
  //Creating the move groups
  moveit::planning_interface::MoveGroupInterface head_group(head_PlanningGroupName);

  //Get the current state of the head
  current_state = head_group.getCurrentState();

  //Get the current state of the joint values
  const robot_state::JointModelGroup *joint_model_group = head_group.getCurrentState()->getJointModelGroup(head_PlanningGroupName);
  current_state->copyJointGroupPositions(joint_model_group, head_jointGroupPositions);

  //Set joint values
  head_jointGroupPositions[0] = head_pan;
  head_jointGroupPositions[1] = head_tilt;
  head_group.setJointValueTarget(head_jointGroupPositions);

  //Set goal tolerance
  head_group.setGoalJointTolerance(0.02);

  //Plan the move
  success = (head_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //Return false if planning was unsuccessful
  if(success == false)
    return false;

  //Move the head
  success = (head_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  return success;
}

/*******************************/
//The function to move the PR2's torso by setting the joint values. It returns a boolean value to indicate if the movement was successful
bool RobotMovement::moveTorsoJoint(float torso_lift)
{
  //Creating the move groups
  moveit::planning_interface::MoveGroupInterface torso_group(torso_PlanningGroupName);

  //Get the current state of the torso
  current_state = torso_group.getCurrentState();

  //Get the current state of the joint value
  const robot_state::JointModelGroup *joint_model_group = torso_group.getCurrentState()->getJointModelGroup(torso_PlanningGroupName);
  current_state->copyJointGroupPositions(joint_model_group, torso_jointGroupPositions);

  //Set the joint value
  torso_jointGroupPositions[0] = torso_lift;
  torso_group.setJointValueTarget(torso_jointGroupPositions);

  //Set goal tolerance
  torso_group.setGoalJointTolerance(0.05);

  //Plan the move
  success = (torso_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //Return false if planning was unsuccessful
  if(success == false)
    return false;

  //Move the head
  success = (torso_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  return success;
}

/*******************************/
//The function to move the hand to the default pose. The default hand is the left. It returns a boolean value to indicate if the movement was successful
bool RobotMovement::moveHandToDefaultPose(atomic<bool> & isInProgess, bool rightHand, const string planningGroupName, const string defaultPoseName)
{
  //Lock the public functions mutex
  publicFunctions_Lock.lock();

  //Indicate if the operation is in progress
  moveInProgress_lock.lock();
  isInProgess = true;

  //Declare a local return value
  bool success;

  //Creating the move groups
  moveit::planning_interface::MoveGroupInterface hand_group(planningGroupName);

  //Read the default pose
  hand_defaultPose = hand_group.getNamedTargetValues(defaultPoseName);

  //Check if default values are read correctly
  if(hand_defaultPose.empty())
    {
      cerr << "\nCould not read the default pose for the PR2's hand.\nYou need to define a pose named \"" << defaultPoseName << "\" in the Moveit setup assistant. To do that you first need to do the following:\n\t. Active_Perception/src/active_perception/PR2GazeboWorkspace/devel/setup.bash\n\troslaunch moveit_setup_assistant setup_assistant.launch\n\tand choose the following path to load:\n\t\tActive_Perception/src/active_perception/PR2GazeboWorkspace/src/moveit_pr2/pr2_moveit_config\n"<<endl;
      exit(EXIT_FAILURE);
    }

  //Return right hand joint values if it is requested in the function call
  if(rightHand)
    success = moveHandJoints(hand_defaultPose.find("r_shoulder_pan_joint")->second, hand_defaultPose.find("r_shoulder_lift_joint")->second, hand_defaultPose.find("r_upper_arm_roll_joint")->second, hand_defaultPose.find("r_elbow_flex_joint")->second, hand_defaultPose.find("r_forearm_roll_joint")->second, planningGroupName);
  else
    //Call the move function for the left hand and return
    success = moveHandJoints(hand_defaultPose.find("l_shoulder_pan_joint")->second, hand_defaultPose.find("l_shoulder_lift_joint")->second, hand_defaultPose.find("l_upper_arm_roll_joint")->second, hand_defaultPose.find("l_elbow_flex_joint")->second, hand_defaultPose.find("l_forearm_roll_joint")->second, planningGroupName);

  //Indicate if the operation is not in progress anymore, successful or unsuccessful
  isInProgess = false;
  moveInProgress_lock.unlock();

  //Unlock the public functions mutex
  publicFunctions_Lock.unlock();

  //Return if the movement was successful
  return success;
}

/*******************************/
//The function to move the PR2's head to a default pose
bool RobotMovement::moveHeadToDefaultPose(const string defaultPoseName)
{
  //Creating the move groups
  moveit::planning_interface::MoveGroupInterface head_group(head_PlanningGroupName);

  //Read the default pose
  head_defaultPose = head_group.getNamedTargetValues(defaultPoseName);

  //Check if default values are read correctly
  if(head_defaultPose.empty())
    {
      cerr << "\nCould not read the default pose for the PR2's hand.\nYou need to define a pose named \"" << defaultPoseName << "\" in the Moveit setup assistant. To do that you first need to do the following:\n\t. Active_Perception/src/active_perception/PR2GazeboWorkspace/devel/setup.bash\n\troslaunch moveit_setup_assistant setup_assistant.launch\n\tand choose the following path to load:\n\t\tActive_Perception/src/active_perception/PR2GazeboWorkspace/src/moveit_pr2/pr2_moveit_config\n"<<endl;
      exit(EXIT_FAILURE);
    }

  //Call the function to move the head and return
  return moveHeadJoints(head_defaultPose.find("head_pan_joint")->second, head_defaultPose.find("head_tilt_joint")->second);
}

/*******************************/
//The function to move the PR2's torso to a default pose
bool RobotMovement::moveTorsoToDefaultPose(const string defaultPoseName)
{
  //Creating the move groups
  moveit::planning_interface::MoveGroupInterface torso_group(torso_PlanningGroupName);

  //Read the default pose
  torso_defaultPose = torso_group.getNamedTargetValues(defaultPoseName);

  //Check if default values are read correctly
  if(torso_defaultPose.empty())
    {
      cerr << "\nCould not read the default pose for the PR2's hand.\nYou need to define a pose named \"" << defaultPoseName << "\" in the Moveit setup assistant. To do that you first need to do the following:\n\t. Active_Perception/src/active_perception/PR2GazeboWorkspace/devel/setup.bash\n\troslaunch moveit_setup_assistant setup_assistant.launch\n\tand choose the following path to load:\n\t\tActive_Perception/src/active_perception/PR2GazeboWorkspace/src/moveit_pr2/pr2_moveit_config\n"<<endl;
      exit(EXIT_FAILURE);
    }

  //Call the function to move the torso and return
  return moveTorsoJoint(torso_defaultPose.find("torso_lift_joint")->second);
}

/*******************************/
//The function to set collision objects
//It sets the collision objects for the left and right hands of the robot only
void RobotMovement::defineCollisionObjects()
{
  //Create temporary move groups for the left and right hands
  moveit::planning_interface::MoveGroupInterface leftHand_group(leftHand_PlanningGroupName);
  moveit::planning_interface::MoveGroupInterface rightHand_group(rightHand_PlanningGroupName);

  //Define the collision object frame identifier for the left hand
  collision_object.header.frame_id = leftHand_group.getPlanningFrame();

  //Define name of the object
  collision_object.id = "Table&Objects";

  //Define the object avoidance box
  objectAvoidanceBox.type = objectAvoidanceBox.BOX;
  objectAvoidanceBox.dimensions.resize(3);
  objectAvoidanceBox.dimensions[0] = AVOIDANCE_AREA_WIDTH + AVOIDANCE_AREA_TOLERANCE;
  objectAvoidanceBox.dimensions[1] = AVOIDANCE_AREA_LENGTH + AVOIDANCE_AREA_TOLERANCE;
  objectAvoidanceBox.dimensions[2] = AVOIDANCE_AREA_HEIGHT + AVOIDANCE_AREA_TOLERANCE;

  //Define the pose for the box
  box_pose.orientation.w = 1.0;
  if(SIMULATION_WORLD_ENABLE) //The robot is assumed in different locations in the world in simulation and real world
    box_pose.position.x = AVOIDANCE_AREA_CENTER_WIDTH;
  else
    box_pose.position.x = 2.2 * AVOIDANCE_AREA_CENTER_WIDTH;
  box_pose.position.y = 0;
  box_pose.position.z = AVOIDANCE_AREA_HEIGHT / 2;

  //Load the box into the collision object
  collision_object.primitives.push_back(objectAvoidanceBox);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  //Load the collision object from the point of the left hand frame into the vector of collision objects
  collision_objects.push_back(collision_object);

  //Define the collision object frame identifier for the right hand
  collision_object.header.frame_id = rightHand_group.getPlanningFrame();

  //Load the collision object from the point of the right hand frame into the vector of collision objects
  collision_objects.push_back(collision_object);

  //Add the collision objects to the world
  planning_scene_interface.addCollisionObjects(collision_objects);
  ros::Duration(1.0).sleep(); //To allow the Movegroup to receive and process the collision object

  return;
}

