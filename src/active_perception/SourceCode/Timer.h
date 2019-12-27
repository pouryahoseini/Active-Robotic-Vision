/*
 * Timer.h
 *
 *  Created on: Aug 26, 2016
 *      Author: pourya
 */

//Guard
#ifndef TIMER_H_
#define TIMER_H_

//Headers
#include <ctime>
#include <opencv2/highgui.hpp>
#include <iostream>

//Global definitions
#include "GlobalDefinitions.h"

//Namespaces being used
using namespace std;
using namespace cv;

//The class used for computing frame rate (frame per second)
class Timer
{
public:
  //Constructor
  Timer();

  //The function to get current frame rate (FPS)
  int getFrameRate();

  //The function to overlay frame rate on input camera frames
  void overlayFrameRate_CameraStream(const char *title);

private:
  time_t start, end, start2;
  long iteration=0, iteration2=0;
  int frameRate;
  stringstream frameRate_string;
};


#endif /* TIMER_H_ */
