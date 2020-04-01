/*
 * Planning.h
 *
 *  Created on: Oct 14, 2018
 *      Author: pourya
 */

#ifndef SOURCECODE_PLANNING_H_
#define SOURCECODE_PLANNING_H_

//Headers
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#include <iostream>
#include <string>
#include <sstream>

#include "opencv2/opencv.hpp"

#include <Python.h>
#include <boost/python.hpp>
#include <boost/numpy.hpp>

#include "GlobalDefinitions.h"
#include "Localization3D.h"

//Namespaces being used
using namespace std;
using namespace cv;
namespace bp = boost::python;
namespace np = boost::numpy;


//Class to implement the planner class
class Planning
{
public:
  //Constructor
  Planning();

  //The function to plan the hand movement. It returns true if the planning was successful. It returns true if the planning was successful.
  bool plan(float & shoulder_pan, float & shoulder_lift, float & upper_arm_roll, float & elbow_flex, float & forearm_roll, vector<objectInfo> mainCameraObjectsInfo);

  //The function to plan hand movement based on triangulations. Only three joints are planned; the other two are fixed. It returns true if the planning was successful.
  bool deterministically_triangulation_planning(float & shoulder_pan, float & shoulder_lift, float & upper_arm_roll, float & elbow_flex, float & forearm_roll, vector<objectInfo> mainCameraObjectsInfo);

private:
  int objectCounter;
  float objectX, objectY, objectZ;
  string camera1_frameID;
  bool planningSuccessful;

  //Deterministic triangulation planning
  tf::StampedTransform frameTransformation;
  Localization3D localizationInstance;
  tf::Vector3 framesRelativePosition;
  float frameX, frameY, frameZ, complementary_objectArmAngle_horizontal, complementary_objectArmAngle_vertical;
  Point3d locationWRTBase;
  bool foundObjectOfInterest;
};

#endif /* SOURCECODE_PLANNING_H_ */
