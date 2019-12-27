/*
 * Preprocessing.h
 *
 *  Created on: Aug 15, 2018
 *      Author: pourya
 */

//Guard
#ifndef SOURCECODE_PREPROCESSING_H_
#define SOURCECODE_PREPROCESSING_H_

//Headers
#include <cstdio>
#include <iostream>

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <ros/ros.h>

#include "GlobalDefinitions.h"
#include "ROS.h"

//Namespaces being used
using namespace std;
using namespace cv;

//Global definitions for camera types
extern const int MAIN_CAMERA, SECONDARY_CAMERA;

//Class to preprocess images
class Preprocessing
{
public:
  //Function preprocess images
  void preprocess(Mat BGRImage[], Mat & depthImage, Mat originalBGRImage[], Mat & originalDepthImage);

  //Function to wait until proper images are retrieved. In other words wait until ROS is "warmed up"
  /* Note: Not getting proper frames from actual robot cameras in a ROS system is common in practice */
  void waitForProperFrames(ROSImageSubscriber * ROSBGRSubscriber1, ROSImageSubscriber * ROSBGRSubscriber2, ROSImageSubscriber * ROSDepthSubscriber);


private:
  //Function to perform histogram equalization on the input image
  void Histogram_Equalization(Mat & input_image);

  Mat BGRImage[2], depthImage, HSVImage, HSVChannels[3];
  cv::cuda::GpuMat intensityChannel_GPU;
};

//Class to compare a frame with the previous frame
struct FrameComparison
{
  //Constructor
  FrameComparison()
  {
    //Set an initial reduced image for the former frame, used in the compareFrames function
    formerReducedVector = Mat::zeros(Size(IMAGE_WIDTH, 1), CV_32SC1);
  }

  //Function to compare a frame with the previous frame. It returns false if the current frame is the same as the previous frame
  bool compareFrames(Mat & current_frame);

private:
  Mat differenceVector, grayImage, reducedVector, formerReducedVector;
  int numberOfNonZero;
  bool framesEqual;
};

#endif /* SOURCECODE_PREPROCESSING_H_ */
