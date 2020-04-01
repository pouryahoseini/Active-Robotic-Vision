/*
 * Preprocessing.cpp
 *
 *  Created on: Aug 15, 2018
 *      Author: pourya
 */

//Header
#include "Preprocessing.h"

/******************************/
//Function preprocess images
void Preprocessing::preprocess(Mat BGRImage[], Mat & depthImage, Mat originalBGRImage[], Mat & originalDepthImage)
{
  //Resizing input images
  resize(depthImage, depthImage, Size(IMAGE_WIDTH, IMAGE_HEIGHT));
  resize(BGRImage[MAIN_CAMERA], BGRImage[MAIN_CAMERA], Size(IMAGE_WIDTH, IMAGE_HEIGHT));
  if(ROS_IMAGE2_ENABLE == true)
    resize(BGRImage[SECONDARY_CAMERA], BGRImage[SECONDARY_CAMERA], Size(IMAGE_WIDTH, IMAGE_HEIGHT));

  //If running simulation, convert RGB to BGR to fix the arm camera problem with the PR2 model in Gazebo
  if(SIMULATION_WORLD_ENABLE && ROS_IMAGE2_ENABLE)
    cvtColor(BGRImage[SECONDARY_CAMERA], BGRImage[SECONDARY_CAMERA], COLOR_RGB2BGR);

  //Preserving original images
  originalDepthImage = depthImage.clone();
  BGRImage[MAIN_CAMERA].copyTo(originalBGRImage[MAIN_CAMERA]);
  if(ROS_IMAGE2_ENABLE == true)
    originalBGRImage[SECONDARY_CAMERA] = BGRImage[SECONDARY_CAMERA].clone();

  //Convert and scale depth image
  depthImage.convertTo(depthImage, CV_8U, (1.0/50));
  originalDepthImage.convertTo(originalDepthImage, CV_8U);

  //Equalize histogram of input images, if enabled
  if(PREPROCESSING_EQUALIZE_HISTOGRAM)
    {
      Histogram_Equalization(BGRImage[MAIN_CAMERA]);
      if(ROS_IMAGE2_ENABLE == true)
	Histogram_Equalization(BGRImage[SECONDARY_CAMERA]);
    }

  //Denoising input images by Gaussian filter, if enabled
  if(PREPROCESSING_BLUR_TEST_IMAGES)
    {
      GaussianBlur(BGRImage[MAIN_CAMERA], BGRImage[MAIN_CAMERA], Size(5, 5), 2);
      if(ROS_IMAGE2_ENABLE == true)
	GaussianBlur(BGRImage[SECONDARY_CAMERA], BGRImage[SECONDARY_CAMERA], Size(5, 5), 2);
    }

  //If mid-process images are enabled to display, show original depth image
  if (DISPLAY_MID_PROCESS_FRAMES == true)
    imshow("pre-Depth", depthImage);

  return;
}

/******************************/
//Function to wait until proper images are retrieved. In other words wait until ROS is "warmed up"
/* Note: Not getting proper frames from actual robot cameras in a ROS system is common in practice */
void Preprocessing::waitForProperFrames(ROSImageSubscriber * ROSBGRSubscriber1, ROSImageSubscriber * ROSBGRSubscriber2, ROSImageSubscriber * ROSDepthSubscriber)
{
  //Print an informative message to the user
  printf("\n\nWaiting to retrieve proper size frames for the first time.\n");

  //Capture the frames initially
  BGRImage[MAIN_CAMERA] = ROSBGRSubscriber1->getImageFromROS();
  if (ROS_IMAGE2_ENABLE == true)
    BGRImage[SECONDARY_CAMERA] = ROSBGRSubscriber2->getImageFromROS();
  else
    BGRImage[SECONDARY_CAMERA] = Mat(Size(1,1), CV_8UC3);
  depthImage = ROSDepthSubscriber->getImageFromROS();

  //Continue looping until all the frames are in the correct form
  while((BGRImage[MAIN_CAMERA].channels() != 3) || (BGRImage[MAIN_CAMERA].empty()) || (BGRImage[SECONDARY_CAMERA].channels() != 3) || (BGRImage[SECONDARY_CAMERA].empty()) ||(depthImage.channels() != 1) || (depthImage.empty()))
    {
      // cout<<endl<<BGRImage[MAIN_CAMERA].channels()<<"   "<<BGRImage[SECONDARY_CAMERA].channels()<<"   "<<BGRImage[MAIN_CAMERA].empty()<<"   "<<BGRImage[SECONDARY_CAMERA].empty()<<"   "<<depthImage.channels()<<"    "<<depthImage.empty()<<endl;

      //Spin ROS once
      ros::spinOnce();

      //Capture frames again
      BGRImage[MAIN_CAMERA] = ROSBGRSubscriber1->getImageFromROS();
      if (ROS_IMAGE2_ENABLE == true)
	BGRImage[SECONDARY_CAMERA] = ROSBGRSubscriber2->getImageFromROS();
      depthImage = ROSDepthSubscriber->getImageFromROS();
    }

  //Print an informative message to the user
  printf("Proper size frames retrieved. Continuing...\n\n");

  return;
}

/******************************/
//Function to perform histogram equalization on the input image
void Preprocessing::Histogram_Equalization(Mat & input_image)
{
  //Convert the image to HSV color domain
  cvtColor(input_image, HSVImage, CV_BGR2HSV);

  //Perform histogram equalization on the intensity channel
  split(HSVImage, HSVChannels);
  if(ENABLE_OPENCV_CUDA)
    {
      intensityChannel_GPU.upload(HSVChannels[2]);
      cv::cuda::equalizeHist(intensityChannel_GPU, intensityChannel_GPU);
      intensityChannel_GPU.download(HSVChannels[2]);
    }
  else
    equalizeHist(HSVChannels[2], HSVChannels[2]);

  //Update the original BGR image
  merge(HSVChannels, 3, HSVImage);
  cvtColor(HSVImage, input_image, CV_HSV2BGR);

  return;
}

/************************************* Frame Comparison **************************************/
/******************************/
//Function to compare a frame with the previous frame. It returns false if the current frame is the same as the previous frame
bool FrameComparison::compareFrames(Mat & current_frame)
{
  //Convert the input image to a gray scale one
  cvtColor(current_frame, grayImage, CV_BGR2GRAY);

  //Reduce the gray image to a horizontal vector by summing along the columns
  cv::reduce(grayImage, reducedVector, 0, CV_REDUCE_SUM, CV_32SC1);

  //Compare the current reduced image with the former one
  absdiff(reducedVector, formerReducedVector, differenceVector);
  numberOfNonZero = countNonZero(differenceVector);
  if(numberOfNonZero == 0)
    framesEqual = false;
  else
    framesEqual = true;

  //Overwrite the current reduced frame on the previous one
  formerReducedVector = reducedVector.clone();

  return framesEqual;
}

