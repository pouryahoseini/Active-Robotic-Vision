/*
 * Timer.cpp
 *
 *  Created on: Aug 26, 2016
 *      Author: pourya
 */

//Headers
#include "Timer.h"

/******************************/
//Defining the constructor function
Timer::Timer()
{
  time(&start);
}

/******************************/
//Defining the function to get current frame rate (FPS)
int Timer::getFrameRate()
{
  iteration++;
  iteration2++;
  time(&end);

  if (iteration == 250)
    {
      start = start2;
      iteration = iteration2;
    }
  if (iteration2 == 150)
    {
      time(&start2);
      iteration2 = 0;
    }

  frameRate = iteration/difftime(end,start);
  return frameRate;
}

/******************************/
void Timer::overlayFrameRate_CameraStream(const char *title)
{
  //printf("%d\n", frameRate);
  frameRate_string.str(string());
  frameRate_string << "Frame Rate (FPS): " << frameRate;
  //putText(depthImage2, frameRate_string.str(), Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 200));

  displayOverlay(title, frameRate_string.str());
}
