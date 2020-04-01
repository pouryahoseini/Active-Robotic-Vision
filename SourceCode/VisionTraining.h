/*
 * VisionTraining.h
 *
 *  Created on: Mar 27, 2017
 *      Author: pourya
 */

//Guard
#ifndef SRC_TRAINER_H_
#define SRC_TRAINER_H_

//Headers
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>

#include <Python.h>
#include <boost/python.hpp>
#include <boost/numpy.hpp>

#include "FeatureDescriptor.h"
#include "FeatureReduction.h"
#include "GlobalDefinitions.h"
#include "PythonCppConvertor.h"

//Global definitions

//Namespaces being used
using namespace std;
using namespace cv;
using namespace cv::ml;
namespace bp = boost::python;
namespace np = boost::numpy;

//The class to alter training samples to create added artificial training samples
class dataAugmentation
{
public:
  //The function to add Gaussian noise to the input image
  Mat & addNoise(Mat & input_image, int HSVChannel, float standardDeviation = 0.1);

  //The function to blur images with directional or omni-directional Gaussian filter
  Mat & blurSample(Mat & input_image, int blurMode);

  //The function to crop and zoom in the training images
  Mat & cropSample(Mat & input_image, int cropMode);

  //The function to changes brightness of a sample image
  Mat & changeBrightness(Mat & input_image, int brightnessMode);

  //The function to transform images geometrically
  Mat & transformGeometrically(Mat & input_image, int transformType);

private:
  Mat processedImage, separatedChannels[3], noise, floatImage, HSVImage, HSVChannels[3];
  double maxPixelValue, minPixelValue;
  Size imageSize;
};

//The class containing all the required training functions
struct VisionTraining
{
  //Constructor
  VisionTraining(classifierTypes training_method = classifierTypes::SVM);

  //The function to run the training
  void Train();

private:
  //SVM Learning
  void SupportVectorMachine();

  //Function to get the next training sample, returns true if training should continue
  bool getNextTrainingSample(Mat & output_trainnigSample);

  //The function to sweep the full size training files, returns true if training should continue
  bool fetchTrainingImage(Mat & output_trainingImage);

  //The function to write the list of objects into a file
  void ObjectListWriter();

  //The function to read the list of objects from a text file
  void objectListReader();

  string objectName, classifierFolderName, classificationSourceDirectory;
  stringstream fileName, trainingFileName;
  FileStorage labelsBook;
  ifstream objectsList, annotationFile;
  int numberOfObjects = 0, sampleCounter, fileCounter, folderCounter, currentFolderCounter;
  int noiseAddCounter = 1, blurringCounter = 0, cropCounter = 0, brightnessCounter = 0, geoTransformCounter = 0;
  vector<string> objects, sampleLabels;
  Mat trainingImage, HOG, sampleLabelCodes, sampleColorHistMat, sampleHOGMat, sampleFeatureMatrix, reducedHOG, colorHistogram;
  Mat copiedOriginalSample;
  bool continueTraining, trainingSampleSkip, emptyImage, fetchOriginalSample = true;
  char objectNumber[10];
  FeatureDescriptor features;
  FeatureReduction HOG_featureReduction;
  classifierTypes trainingMethod;
  bp::object ClassifierModule_BoostObject, featureClassifierClass_BoostObject;
  bp::list scikitPythonArgs;
  Python_Cpp_Convertor<float> floatMatNdarrayConvertor;
  Python_Cpp_Convertor<int> intMatNdarrayConvertor;
  dataAugmentation sampleAugmentation;
};

#endif /* SRC_TRAINER_H_ */
