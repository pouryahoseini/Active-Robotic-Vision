/*
 * ConfidenceCheck.h
 *
 *  Created on: Oct 11, 2017
 *      Author: pourya
 */

//Guard
#ifndef SOURCECODE_CONFIDENCECHECK_H_
#define SOURCECODE_CONFIDENCECHECK_H_

//Headers
#include <iostream>
#include <cstdio>
#include <cstdlib>

#include <opencv2/core.hpp>

//Global definitions
#include "GlobalDefinitions.h"

//Namespaces being used
using namespace std;
using namespace cv;

//The structure containing functions to check confidence of a classifier's output probabilities
struct ConfidenceCheck
{
  //constructor
  ConfidenceCheck();

  //Function to check the confidence of a classification
  bool checkConfidence(Mat & inputClassProbabilities, double unknownProbability, float * output_score = nullptr);

  //Overloaded function to check confidence when it is based on only the class probabilities
  bool checkConfidence(Mat & inputClassProbabilities, float * output_score = nullptr);

  //Overloaded function to check confidence when it is based on the Dempster-Shafer-based unknown probability only
  bool checkConfidence(double unknownProbability, float * output_score = nullptr);

private:
  //The function to check the first max score divided by the second max score
  bool firstMaxOverSecondMaxConfidence(Mat & inputClassProbabilities, float firstMaxOverSecondMaxThreshold = CONFIDENCE_FIRST_MAX_TO_SECOND_MAX_RATIO, float * output_score = nullptr);

  //Function to check unknown probability in a Dempster-Shafer-based classification against a threshold
  bool unknownProbabilityConfidence(double unknownProbability, float * output_score = nullptr);

  bool confirmed;
  double firstMax, secondMax, minScore, firstToSecondMaxRatio;
  float firstMaxOverSecondMaxThreshold, score;
  Point minLocation, maxLocation;
  Mat mask;
};

#endif /* SOURCECODE_CONFIDENCECHECK_H_ */
