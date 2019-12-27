/*
 * DataFusion.h
 *
 *  Created on: Oct 26, 2017
 *      Author: pourya
 */

//Guard
#ifndef SOURCECODE_DATAFUSION_H_
#define SOURCECODE_DATAFUSION_H_

//Headers
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <opencv2/opencv.hpp>

//Global definitions
#include "GlobalDefinitions.h"

//Namespaces being used
using namespace std;
using namespace cv;

//The class to fuse data
class DataFusion
{
public:
  //Function to fuse probability vectors. In the case of the Dempster-Shafer decision fusion, the last two input arguments are actually used, else they have no effect on the output
  Mat & Fuse(Mat & inputProbabilities_1, Mat & inputProbabilities_2, double & allClassesProbability_1, double & allClassesProbability_2);

  //Overloaded function for fusing probability vectors as well as the unknown probabilities in the case of a Dempster-Shafer fusion
  Mat & Fuse(Mat & inputProbabilities_1, Mat & inputProbabilities_2, double & allClassesProbability_1, double & allClassesProbability_2, double & output_unknowProbability);

private:
  /*The function for Bayesian decision fusion
   * The evidence is considered equal for any data sample coming from two cameras
   * The priors are taken equal for every class label */
  Mat & BayesianDecisionFusion(Mat & inputProbabilities_1, Mat & inputProbabilities_2);

  /* The function for Unnormalized Dempster-Shafer decision fusion with pignistic probabilities as output
   * Probability of category "all objects" is the probability of belonging to any of the categories */
  Mat & UnnormalizedDempsterShaferDecisionFusion(Mat & inputProbabilities_1, Mat & inputProbabilities_2, double & allClassesProbability_1, double & allClassesProbability_2);

  //Overloaded function to additionally compute the output unknown probability after the Dempster-Shafer fusion
  Mat & UnnormalizedDempsterShaferDecisionFusion(Mat & inputProbabilities_1, Mat & inputProbabilities_2, double & allClassesProbability_1, double & allClassesProbability_2, double & output_unknowProbability);

  Mat multipliedProbabilities, fusedProbabilities;
  int counter;
  float probabilitySum;
};

#endif /* SOURCECODE_DATAFUSION_H_ */
