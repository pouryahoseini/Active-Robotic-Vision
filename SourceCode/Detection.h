/*
 * Detector.h
 *
 *  Created on: Jul 24, 2018
 *      Author: pourya
 */

//Guard
#ifndef SOURCECODE_DETECTION_H_
#define SOURCECODE_DETECTION_H_

//Headers
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <iostream>
#include <string>
#include <sstream>

#include "opencv2/opencv.hpp"
#include <opencv2/tracking.hpp>
#include <opencv2/dnn.hpp>

#include <Python.h>
#include <boost/python.hpp>
#include <boost/numpy.hpp>

#include "GlobalDefinitions.h"
#include "Classification.h"
#include "PythonCppConvertor.h"
#include "ConfidenceCheck.h"
#include "TrackingManagement.h"

//Namespaces being used
using namespace std;
using namespace cv;
namespace bp = boost::python;
namespace np = boost::numpy;


//The class to detect objects in an image
class Detection : private Classification
{
public:
  //Constructor
  Detection(detectorTypes detection_method = detectorTypes::Intensity_Thresholding, classifierTypes classification_method = classifierTypes::SVM, int inputImage_Width = IMAGE_WIDTH, int inputImage_Height = IMAGE_HEIGHT);

  //Function to detect objects in an image
  vector<objectInfo> Detect(Mat & input_image, int cameraNumber, Mat * depthImage = nullptr);

  //Declare the winner finder function from the base class as public in the derived class
  using Classification::FindWinnerClass;

private:
  //Function to detect objects based on the sliding window technique
  vector<objectInfo> & SlidingWindow(Mat & input_image, int & cameraNumber, Mat * depthImage = nullptr);

  //Function to detect objects based on the Mixture of Gaussians background subtraction technique
  vector<objectInfo> & BackgroundSubtraction(Mat & input_image, int & cameraNumber, Mat * depthImage = nullptr);

  //Function to detect objects based on intensity thresholding
  vector<objectInfo> & IntensityThresholding(Mat & input_image, int & cameraNumber, Mat * depthImage = nullptr);

  //Function to detect blobs in a foreground map based on the Connected Components technique
  vector<objectInfo> & ConnectedComponents_BlobDetection(Mat & input_BGRImage, Mat & input_foregroundImage, int & cameraNumber);

  //Function to perform non-maximum suppression. It returns the vector of indices of the remaining bounding boxes
  vector<int> NonMaximumSuppression(vector<Rect> boundingBoxes, vector<float> scores, bool discardcloseBBOxes = true, vector<string> labels = {});

  //Function to implement mixture of gaussians background subtraction
  Mat & MixtureOfGaussians(Mat & inputImage);

  //Function to process foreground images
  Mat & ProcessForeground(Mat & input_foreground, int & cameraNumber);

  //Function to detect and classify objects given a foreground map
  vector<objectInfo> & Detect_fromForegroundMap(Mat & input_BGRImage, Mat & input_foregroundImage, int & cameraNumber);

  int objectCounter, cameraCounter, numberOfObjects, countComponents, trackerCounter, winnerIndex;
  stringstream windowName;
  Mat grayImage, objectImage;
  Mat foregroundImage;
  vector<objectInfo> & (Detection::* detectionFunction_ptr) (Mat &, int &, Mat *);
  ConfidenceCheck validateRecognition;
  detectorTypes detectionMethod;
  objectInfo currentBlobInfo;
  vector<objectInfo> detectedObjectsInfo, detectedBlobsInfo;
  float discardRatio;

  //Connected components variables
  int ROI_Height, ROI_Width, ROI_Left, ROI_Top, objectPixelNumber;
  Mat mask, labels, stats, centroids, allObjectsRGB, objectsColored, boundaryMask, inputForeground;
  Rect undiscardedRegion;
  cv::RNG randomColor;

  //Foreground processing variables
  Mat foregroundImage_cleaned, structuringElement;

  //Depth processing variables
  int pixelDepth;
  bool useDepth;
  Mat depthMask1, depthMask2, depthMask, depthPyramid;

  //Intensity threshold based segmentation variables
  int intensityThreshold;
  Mat intensityMask, intensityPyramid;

  //Mixture of Gaussians variables
  Ptr<BackgroundSubtractor> MoGBackSub;
  Mat Mog_foregroundImage;
  int MoG_variableThreshold = MOG_VARIABLE_THRESHOLD;
  int MoG_history = MOG_HISTORY;
  double Mog_learningRate = MOG_LEARNING_RATE_NOMINATOR / (double) MoG_history;
  TrackingManagement trackingManager;

  //Sliding window variables
  int shapeCounter, pyramidLevelCounter, pixelWidth, pixelHeight, numberOfShapes, windowHeight, windowWidth, BBoxCounter, pyrupCounter;
  cv::Size windowSize;
  Mat pyramidImage, objectProbabilities;
  vector<Rect> allBoundingBoxes;
  vector<Mat> allProbabilities;
  Rect currentSlidingWindow;
  vector<int> remainingBBoxIndices;
  vector<double> allUnknownProbabilities;
  double winnerProbability;
  string winnerObjectLabel;
  vector<double> allWinnerProbabilities;
  vector<string> allWinnerLabels;
  vector<float> convertedWinnerProbabilities;

  //Non-maximum suppression variables
  int bbox_counter1, bbox_counter2, x_left[2], x_right[2], y_top[2], y_bottom[2], minDistance, distanceArray[8];
  vector<bool> remainingBBoxUsage;
  vector<int> proximityRefinedBoundingBoxes;
};

#endif /* SOURCECODE_DETECTION_H_ */
