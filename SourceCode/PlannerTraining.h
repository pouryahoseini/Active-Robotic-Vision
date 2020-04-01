/*
 * PlannerTraining.h
 *
 *  Created on: Oct 14, 2018
 *      Author: pourya
 */

#ifndef SOURCECODE_PLANNERTRAINING_H_
#define SOURCECODE_PLANNERTRAINING_H_

//Headers
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <iostream>
#include <string>
#include <sstream>

#include <Python.h>
#include <boost/python.hpp>
#include <boost/numpy.hpp>

#include "opencv2/opencv.hpp"

#include "GlobalDefinitions.h"

//Namespaces being used
using namespace std;
using namespace cv;
namespace bp = boost::python;
namespace np = boost::numpy;


//Class to implement the planner training
class PlannerTraining
{
public:
  //Constructor
  PlannerTraining();

private:

};

#endif /* SOURCECODE_PLANNERTRAINING_H_ */
