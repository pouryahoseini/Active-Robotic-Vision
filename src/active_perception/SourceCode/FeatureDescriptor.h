/*
 * FeatureDescriptor.h
 *
 *  Created on: Apr 2, 2017
 *      Author: pourya
 */

#ifndef SRC_FEATUREDESCRIPTOR_H_
#define SRC_FEATUREDESCRIPTOR_H_

//Headers
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <iostream>
#include <string>
#include <sstream>

#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/cudaobjdetect.hpp>

//Global definitions
#include "GlobalDefinitions.h"

//Namespaces being used
using namespace std;
using namespace cv;

//The class responsible for generating feature descriptors
struct FeatureDescriptor
{
  //Constructor
  FeatureDescriptor();

  //The main function to extract features
  void extractFeatures(Mat & feature_detector_input, Mat & color_histogram, Mat & HOG);

private:
  //Function to compute color histogram of the input image
  void colorHistogram(Mat & input_image, Mat & color_histogram);

  //Function to compute the histogram of oriented gradients of the input image
  void HoG(Mat & input_image, Mat & output_features);

  //HOG
  vector<float> HOG_descriptor;
  cv::cuda::GpuMat gpuImage, gpu_descriptor_temp;
  Ptr<cv::cuda::HOG> gpu_hog;
  HOGDescriptor hog;
  Mat grayImage, resizedImage, convertedGPUImage;

  //Color histogram
  Mat input_image_converted, histogram_2D[2], flattenedHistogram[2];
  int histSize[2], channels_HS[2] = {0, 1}, channels_uv[2] = {1, 2};
  float hueRange[2] = {0, 180}, SaturationRange[2] = {0, 255}, uvRange[2] = {0, 255};
  const float * histRange_Luv[2] = {uvRange, uvRange}, * histRange_HSV[2] = {hueRange, SaturationRange};
};

#endif /* SRC_FEATUREDESCRIPTOR_H_ */
