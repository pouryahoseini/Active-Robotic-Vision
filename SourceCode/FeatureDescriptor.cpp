/*
 * FeatureDescriptor.cpp
 *
 *  Created on: Apr 2, 2017
 *      Author: pourya
 */

//Header file
#include "FeatureDescriptor.h"

/****************************************/
//Constructor
FeatureDescriptor::FeatureDescriptor()
{
  //Initialize the HOG feature descriptor
  if(ENABLE_OPENCV_CUDA)
    gpu_hog = cv::cuda::HOG::create(HOG_WINDOWS_SIZE, HOG_BLOCK_SIZE, HOG_BLOCK_STRIDE, HOG_CELL_SIZE, HOG_BIN_NUMBER);
  else
    hog = HOGDescriptor(HOG_WINDOWS_SIZE, HOG_BLOCK_SIZE, HOG_BLOCK_STRIDE, HOG_CELL_SIZE, HOG_BIN_NUMBER);

  //Setting up the histogram calculator
  histSize[0] = histSize[1] = static_cast<int>(sqrt(COLOR_HISTOGRAM_BIN_NUMBER / 2.0));
}

//The main function to extract features
void FeatureDescriptor::extractFeatures(Mat & feature_detector_input, Mat & color_histogram, Mat & HOG)
{
  //Color histogram
  this->colorHistogram(feature_detector_input, color_histogram);

  //HoG descriptor
  cvtColor(feature_detector_input, grayImage, CV_BGR2GRAY);
  this->HoG(grayImage, HOG);

  return;
}

/****************************************/
//Function to compute color histogram of the input image
void FeatureDescriptor::colorHistogram(Mat & input_image, Mat & color_histogram)
{
  //Calculate the histogram in the L*u*v color space
  cvtColor(input_image, input_image_converted, CV_BGR2Luv);
  calcHist(& input_image_converted, 1, channels_uv, Mat(), histogram_2D[0], 2, histSize, histRange_Luv, true, false);

  //Calculate the histogram in the HSV color space
  cvtColor(input_image, input_image_converted, CV_BGR2HSV);
  calcHist(& input_image_converted, 1, channels_HS, Mat(), histogram_2D[1], 2, histSize, histRange_HSV, true, false);

  //Flatten the 2D histograms
  flattenedHistogram[0] = histogram_2D[0].reshape(1, 1);
  flattenedHistogram[1] = histogram_2D[1].reshape(1, 1);

  //constructing the feature vector
  hconcat(flattenedHistogram, 2, color_histogram);

  //Normalize the feature vector
  normalize(color_histogram, color_histogram, 1, 0, NORM_L2, -1, Mat()); // normalize(color_histogram, color_histogram, 0, 1, NORM_MINMAX, -1, Mat());

  //Convert the histogram according to the requirements
  color_histogram.convertTo(color_histogram, CV_32FC1);

  return;
}

/****************************************/
//Function to compute the histogram of oriented gradients of the input image
void FeatureDescriptor::HoG(Mat & input_gray_image, Mat & output_features)
{
  //Resizing the input image to the pre-specifdied size for HOG
  resize(input_gray_image, resizedImage, HOG_WINDOWS_SIZE);

  //Compute the HOG features
  if(ENABLE_OPENCV_CUDA)
    {
      gpuImage.upload(resizedImage);
      gpu_hog->compute(gpuImage, gpu_descriptor_temp);
      output_features = Mat(gpu_descriptor_temp);
    }
  else
    {
      hog.compute(resizedImage, HOG_descriptor);
      output_features = Mat(1, HOG_descriptor.size(), CV_32FC1, (double *) HOG_descriptor.data());
    }

  //Normalize the features
  normalize (output_features, output_features, 1, 0, NORM_L2, -1, Mat()); // normalize (output_features, output_features, 0, 1, NORM_MINMAX, -1, Mat());

  return;
}

