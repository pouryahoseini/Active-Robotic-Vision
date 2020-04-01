/*
 * DataFusion.cpp
 *
 *  Created on: Oct 26, 2017
 *      Author: pourya
 */

//Headers
#include "DataFusion.h"

/*************************/
//Function to fuse probability vectors. In the case of the Dempster-Shafer decision fusion, the last two input arguments are actually used, else they have no effect on the output
Mat & DataFusion::Fuse(Mat & inputProbabilities_1, Mat & inputProbabilities_2, double & allClassesProbability_1, double & allClassesProbability_2)
{
  if (DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer) //Dempster-Shafer decision fusion
    return UnnormalizedDempsterShaferDecisionFusion(inputProbabilities_1, inputProbabilities_2, allClassesProbability_1, allClassesProbability_2);
  else if(DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::Bayesian) //Bayesian decision fusion
    return BayesianDecisionFusion(inputProbabilities_1, inputProbabilities_2);
}

/*************************/
//Overloaded function for fusing probability vectors as well as the unknown probabilities in the case of a Dempster-Shafer fusion
Mat & DataFusion::Fuse(Mat & inputProbabilities_1, Mat & inputProbabilities_2, double & allClassesProbability_1, double & allClassesProbability_2, double & output_unknowProbability)
{
  if (DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer) //Dempster-Shafer decision fusion
    return UnnormalizedDempsterShaferDecisionFusion(inputProbabilities_1, inputProbabilities_2, allClassesProbability_1, allClassesProbability_2, output_unknowProbability);
  else if(DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::Bayesian) //Bayesian decision fusion
    return BayesianDecisionFusion(inputProbabilities_1, inputProbabilities_2);
}

/*************************/
/* The function for Bayesian decision fusion
 * The evidence is considered equal for any data sample coming from two cameras
 * The priors are taken equal for every class label */
Mat & DataFusion::BayesianDecisionFusion(Mat & inputProbabilities_1, Mat & inputProbabilities_2)
{
  //Multiplying elementwise the two probability vector
  multipliedProbabilities = inputProbabilities_1.mul(inputProbabilities_2);

  //Changing scale so that the sum of probabilities become 1
  fusedProbabilities = multipliedProbabilities * (1.0f / sum(multipliedProbabilities).val[0]);

  //Return the fused probability vector
  return fusedProbabilities;
}

/*************************/
/* The function for Unnormalized Dempster-Shafer decision fusion with pignistic probabilities as output
 * Probability of category "all objects" is the probability of belonging to any of the categories */
Mat & DataFusion::UnnormalizedDempsterShaferDecisionFusion(Mat & inputProbabilities_1, Mat & inputProbabilities_2, double & allClassesProbability_1, double & allClassesProbability_2)
{
  //Compute unnormalized masses
  multipliedProbabilities = inputProbabilities_1.mul(inputProbabilities_2);
  fusedProbabilities = multipliedProbabilities + (allClassesProbability_1 * inputProbabilities_2) + (allClassesProbability_2 * inputProbabilities_1);

  //Compute pignistic probabilities
  fusedProbabilities += (allClassesProbability_1 * allClassesProbability_2) / static_cast<double>(inputProbabilities_1.cols);

  //Changing scale so that the sum of probabilities become 1
  probabilitySum = sum(fusedProbabilities).val[0]; //Saving the sum internally in the class to possibly use it later for the other overloaded functions
  fusedProbabilities = fusedProbabilities * (1.0f / probabilitySum);

  //Return the fused probability vector
  return fusedProbabilities;
}

/*************************/
//Overloaded function to additionally compute the output unknown probability after the Dempster-Shafer fusion
Mat & DataFusion::UnnormalizedDempsterShaferDecisionFusion(Mat & inputProbabilities_1, Mat & inputProbabilities_2, double & allClassesProbability_1, double & allClassesProbability_2, double & output_unknowProbability)
{
  //Call the main overloaded function
  fusedProbabilities = UnnormalizedDempsterShaferDecisionFusion(inputProbabilities_1, inputProbabilities_2, allClassesProbability_1, allClassesProbability_2);

  //Compute the output unknow (all classes) probability
  output_unknowProbability = allClassesProbability_1 * allClassesProbability_2; //Unnormalized Dempster's rule of combination
  output_unknowProbability /= static_cast<double>(inputProbabilities_1.cols); //Pignistic probability

  //Scale the unknown probability so that the sum of all fused probabilities is equal to one
  output_unknowProbability *= (1.0f / (probabilitySum + output_unknowProbability));

  return fusedProbabilities;
}

