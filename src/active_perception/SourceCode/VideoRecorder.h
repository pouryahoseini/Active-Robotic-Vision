/*
 * VideoRecorder.h
 *
 *  Created on: Apr 6, 2017
 *      Author: pourya
 */

//Guard
#ifndef SRC_VIDEORECORDER_H_
#define SRC_VIDEORECORDER_H_

//Headers
#include <iostream>
#include <cstdio>
#include <string>
#include <cstring>
#include <cstdlib>
#include <ctime>

#include "opencv2/opencv.hpp"

//Global definitions
#include "GlobalDefinitions.h"

//Namespaces being used
using namespace std;
using namespace cv;

//The class for recording videos of the scene
class VideoRecorder
{
public:
  //Constructor
  VideoRecorder();

  //The function to record videos
  void recordVideo(int * key, Mat & bgr_image_original, Mat & bgr_image_processed, Mat & depth_image, Mat & bgr_image2_original, Mat & bgr_image2_processed, Mat & bgr_fused_processed, Mat & amalgamated, int image_height = IMAGE_HEIGHT, int image_width = IMAGE_WIDTH);

private:
  //The function to save current time into a string
  void saveTimeToString();

  int codec;
  bool recording_original, recording_processed;
  char fileAddress[100], fileAddress2[100], timeString[50];
  time_t now;
  struct tm *localTime;
  VideoWriter video_recorder1, video_recorder2, video_recorder3, video_recorder4, video_recorder5, video_recorder6, video_recorder7;
  Mat depthImage_colored;
};

#endif /* SRC_VIDEORECORDER_H_ */
