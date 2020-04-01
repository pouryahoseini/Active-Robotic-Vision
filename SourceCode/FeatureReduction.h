/*
 * FeatureReduction.h
 *
 *  Created on: Apr 11, 2017
 *      Author: pourya
 */

//Guard
#ifndef SRC_FEATUREREDUCTION_H_
#define SRC_FEATUREREDUCTION_H_

//Headers
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <iostream>
#include <string>
#include <sstream>

#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>

//Global definitions
#include "GlobalDefinitions.h"

//Namespaces being used
using namespace std;
using namespace cv;

//The class to reduce number of features
class FeatureReduction
{
public:
  //Dummy default constructor
  FeatureReduction() {}

  //Constructor
  FeatureReduction(string reductionMethod, int numberOfFeaturesExtracted, bool isTraining);

  //The function to train the feature reduction algorithm without training labels <**Overloaded function**>
  void trainFeatureReducer(Mat & trainingFeatures);

  //The function to train the feature reduction algorithm with training labels <**Overloaded function**>
  void trainFeatureReducer(Mat & trainingFeatures, Mat & trainingLabels);

  //The function to get te reduced feature vector
  void reduceFeatures(Mat & inputFeatureVector, Mat & outputFeatureVector);

private:
  //Principal Component Analysis's training function
  void PrincipalComponentAnalysisTraining(Mat & trainingFeatures);

  //Linear Discriminant Analysis's training function
  void LinearDiscriminantAnalysisTraining(Mat & trainingFeatures, Mat & trainingLabels);

  //Function to load trained feature reduction algorithms from files
  void loadLearnedAlgorithm();

  PCA PCA_compressor;
  LDA LDA_compressor;
  stringstream filename;
  int numberOfFeatures, savedNumberOfFeatures;
  string reductionType;
  FileStorage saveFilename, loadFilename;
  Mat projectedTraining, reducedFeatures, interimFeatureVector;
};

#endif /* SRC_FEATUREREDUCTION_H_ */
