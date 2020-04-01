/*
 * Planning.cpp
 *
 *  Created on: Oct 14, 2018
 *      Author: pourya
 */

#include "Planning.h"

/********************************/
//Constructor
Planning::Planning()
{
  //Deciding on which camera frame to use depending on a simulation or a real-world run
  camera1_frameID = SIMULATION_WORLD_ENABLE ? CAMERA1_FRAME_ID_SIMULATION : CAMERA1_FRAME_ID_REAL_WORLD;

  //Getting the relative position and angle of the two robot bases for triangulation planning
  if(ACTIVE_VISION_MODE == activeVisionModes::Deterministic)
    {
      //Get the transformation between the base frame and the triangulation base frame
      frameTransformation = localizationInstance.getFrameTransform(TRIANGULATION_PLANNING_BASE_FRAME, ROBOT_BASE_FRAME);

      //Extract the relative position of the triangulation base frame with respect to the robot's base frame
      framesRelativePosition = frameTransformation.getOrigin();
      frameX = (float) framesRelativePosition.getX();
      frameY = (float) framesRelativePosition.getY();
      frameZ = (float) framesRelativePosition.getZ();
    }
}

/********************************/
//The function to plan the hand movement. It returns true if the planning was successful.
bool Planning::plan(float & shoulder_pan, float & shoulder_lift, float & upper_arm_roll, float & elbow_flex, float & forearm_roll, vector<objectInfo> mainCameraObjectsInfo)
{cout<<endl<<"Planning started"<<endl;
  //Choose the right function to call (i.e. intelligent or deterministic)
  if(ACTIVE_VISION_MODE == activeVisionModes::Deterministic) //Deterministic
    planningSuccessful = deterministically_triangulation_planning(shoulder_pan, shoulder_lift, upper_arm_roll, elbow_flex, forearm_roll, mainCameraObjectsInfo);
  else //Intelligent
    ;
cout<<endl<<"Planning done"<<endl;
  return planningSuccessful;
}

/********************************/
//The function to plan hand movement based on triangulations. Only three joints are planned; the other two are fixed. It returns true if the planning was successful.
bool Planning::deterministically_triangulation_planning(float & shoulder_pan, float & shoulder_lift, float & upper_arm_roll, float & elbow_flex, float & forearm_roll, vector<objectInfo> mainCameraObjectsInfo)
{

  //Get the transformation between the base frame and the triangulation base frame
        frameTransformation = localizationInstance.getFrameTransform(TRIANGULATION_PLANNING_BASE_FRAME, ROBOT_BASE_FRAME);

        //Extract the relative position of the triangulation base frame with respect to the robot's base frame
        framesRelativePosition = frameTransformation.getOrigin();
        frameX = (float) framesRelativePosition.getX();
        frameY = (float) framesRelativePosition.getY();
        frameZ = (float) framesRelativePosition.getZ();





  //Find the first uncertain detection
  foundObjectOfInterest = false;
  for(objectCounter = 0; objectCounter < mainCameraObjectsInfo.size(); objectCounter++)
    if(! mainCameraObjectsInfo[objectCounter].isConfirmed)
      if(mainCameraObjectsInfo[objectCounter].position3D.x > frameX) //Check the feasibility of the object location for moving hand toward that
	{
	  foundObjectOfInterest = true;
	  break;
	}

  //Check if any feasible uncertain detection has been found
  if(! foundObjectOfInterest)
    {
      return false;
    }
cout<<endl<<"Label:  "<<mainCameraObjectsInfo[objectCounter].label<<endl<<"frameXYZ: "<<frameX<<"  "<<frameY<<"  "<<frameZ<<endl;
  //Extract the object position with respect to the robot base
  localizationInstance.transformCameraCoordinate(mainCameraObjectsInfo[objectCounter].position3D, locationWRTBase, camera1_frameID, ROBOT_BASE_FRAME);
  objectX = locationWRTBase.x;
  objectY = locationWRTBase.y;
  objectZ = locationWRTBase.z;
cout<<"objectXYZ: "<<objectX<<"  "<<objectY<<"  "<<objectZ<<endl;
  //Computing the complementary horizontal arm angle
  complementary_objectArmAngle_horizontal = atan((frameY - objectY) / (objectX - frameX));

  //Computing the complementary vertical arm angle
  complementary_objectArmAngle_vertical = atan((frameZ - objectZ) / (objectX - frameX));
cout<<"compHV: "<<complementary_objectArmAngle_horizontal<<"  "<<complementary_objectArmAngle_vertical<<endl;
  //Setting the joint values
  shoulder_pan = - complementary_objectArmAngle_horizontal - ELBOW_FIXED_VALUE;
  shoulder_lift = complementary_objectArmAngle_vertical / 2.0;
  upper_arm_roll = UPPER_ARM_ROLL_VALUE;
  elbow_flex = ELBOW_FIXED_VALUE;
  forearm_roll = complementary_objectArmAngle_vertical / 2.0;
cout<<"pan-lift-roll-flex-roll: "<<shoulder_pan<<"  "<<shoulder_lift<<"  "<<upper_arm_roll<<"  "<<elbow_flex<<"  "<<forearm_roll<<endl<<"******************\n"<<endl;
  return true;
}
