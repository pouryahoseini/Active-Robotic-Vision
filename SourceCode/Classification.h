/*
 * MyClassifier.h
 *
 *  Created on: Mar 28, 2017
 *      Author: pourya
 */

//Guard
#ifndef SRC_CLASSIFICATION_H_
#define SRC_CLASSIFICATION_H_

//Headers
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <climits>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>

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


//The class containing all the required classifiers
struct Classification
{
  //Constructor
  Classification(classifierTypes classification_method = classifierTypes::SVM);

  //The function to classify that returns probabilities of each class
  Mat & Classify(Mat & classification_input);

  //The function to get the probability of category "Unknown Object"
  double & getAllObjectsProbability();

  //The function to find the maximum score and winner label out of a class probability vector
  void FindWinnerClass(Mat & inputClassProbabilities, string & outputClassificationLabel, double & outputEstimatedLabelScore, int * output_winnerIndex = NULL);

protected:
  classifierTypes classificationMethod;

private:
  //Support Machine Classifier (SVM) Classifier
  Mat & SupportVectorMachine(Mat & input_image);

  //The function to load the learning memory into the classifier
  void loadLearningMemory();

  Mat testRegion, detectedLabel, HOG, featureVector, reducedHOG, estimatedScores, colorHistogram;
  Point minLocation, maxLocation;
  double minMat, maxMat, allObjectsProbability;
  string classificationLabel, classifierFolderName, classificationSourceDirectory;
  vector<string> objectNamesList;
  FeatureDescriptor features;
  FeatureReduction HOG_featureReduction;
  bp::object ClassifierModule_BoostObject, featureClassifierClass_BoostObject, classificationResult_Object;
  bp::list scikitPythonArgs, PythonArgs;
  Python_Cpp_Convertor<float> floatMatNdarrayConvertor;
  Python_Cpp_Convertor<double> doubleMatNdarrayConvertor;
};

//The function to read the list of objects from a file. Returns the number of objects
int ObjectListReader(vector<string> & output_objectNamesList);

#endif /* SRC_CLASSIFICATION_H_ */
