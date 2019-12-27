/*
 * ConfidenceCheck.cpp
 *
 *  Created on: Oct 11, 2017
 *      Author: pourya
 */

//Headers
#include "ConfidenceCheck.h"

/*************************************/
//constructor
ConfidenceCheck::ConfidenceCheck()
{
  //Check if the unknown probability is the metric for confidence measurement, the fusion methid be set to Dempster-Shafer
  if((CONFIDENCE_MEASURE == CONFIDENCE_MEASURES::unknownProbability) && (DECISION_FUSION_TYPE != DECISION_FUSION_TYPES::DempsterShafer))
    {
      fprintf(stderr, "\n\nThe confidence measure is set to thresholding the unknown probability, but the decision fusion method is not set to Dempster-Shafer.\n\n");
      exit(EXIT_FAILURE);
    }

  //Store the first max to the second max ratio
  firstToSecondMaxRatio = CONFIDENCE_FIRST_MAX_TO_SECOND_MAX_RATIO;
}

/*************************************/
//Function to check the confidence of a classification
bool ConfidenceCheck::checkConfidence(Mat & inputClassProbabilities, double unknownProbability, float * output_score)
{
  //Decide on which type of confidence measure to use
  if(CONFIDENCE_MEASURE == CONFIDENCE_MEASURES::firstMaxOverSecondMax)
    confirmed = checkConfidence(inputClassProbabilities, output_score);
  else if(CONFIDENCE_MEASURE == CONFIDENCE_MEASURES::unknownProbability)
    confirmed = checkConfidence(unknownProbability, output_score);

  return confirmed;
}

/*************************************/
//Overloaded function to check confidence when it is based on only the class probabilities
bool ConfidenceCheck::checkConfidence(Mat & inputClassProbabilities, float * output_score)
{
  confirmed = firstMaxOverSecondMaxConfidence(inputClassProbabilities, firstToSecondMaxRatio, output_score);

  return confirmed;
}

/*************************************/
//Overloaded function to check confidence when it is based on the Dempster-Shafer-based unknown probability only
bool ConfidenceCheck::checkConfidence(double unknownProbability, float * output_score)
{
  confirmed = unknownProbabilityConfidence(unknownProbability, output_score);

  return confirmed;
}

/*************************************/
//The function to check the first max score divided by the second max score
bool ConfidenceCheck::firstMaxOverSecondMaxConfidence(Mat & inputClassProbabilities, float firstMaxOverSecondMaxThreshold, float * output_score)
{
  //finding the maximum score
  minMaxLoc(inputClassProbabilities, & minScore, & firstMax, & minLocation, & maxLocation);

  //creating the mask with the location of the maximum zeroed out
  mask = Mat::ones(inputClassProbabilities.size(), CV_8UC1);
  mask.at<uchar>(maxLocation) = 0;

  //finding the second max score
  minMaxLoc(inputClassProbabilities, & minScore, & secondMax, & minLocation, & maxLocation, mask);

  //checking the confidence to the class with maximum score
  score = firstMax / secondMax;
  if (score > firstMaxOverSecondMaxThreshold)
    confirmed = true;
  else
    confirmed = false;

  //Output the score
  if(output_score != nullptr)
    * output_score = score;

  //return the boolean variable to show the current detected class is confirmed or not
  return confirmed;
}

/*************************************/
//Function to check unknown probability in a Dempster-Shafer-based classification against a threshold
bool ConfidenceCheck::unknownProbabilityConfidence(double unknownProbability, float * output_score)
{
  //Check the unknown probability against a threshold
  if(unknownProbability >= CONFIDENCE_UNKNOWN_PROBABILITY_THRESHOLD)
    {
      if(output_score != nullptr)
	* output_score = (float) unknownProbability;
      return true;
    }
  else
    return false;
}
