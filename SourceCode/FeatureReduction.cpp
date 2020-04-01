/*
 * FeatureReduction.cpp
 *
 *  Created on: Apr 11, 2017
 *      Author: pourya
 */

//Header
#include "FeatureReduction.h"

/**************************************/
//Constructor
FeatureReduction::FeatureReduction(string reductionMethod, int numberOfFeaturesExtracted, bool isTraining)
{
  if( (reductionMethod == "PCA") || (reductionMethod == "pca") )
    reductionType = "PCA";
  else if( (reductionMethod == "LDA") || (reductionMethod == "lda") )
    reductionType = "LDA";
  else if(reductionMethod == "PCA->LDA")
    reductionType = "PCA->LDA";
  else
    {
      cerr << "\n\nFeature reduction method unknown.\nExiting...\n" << endl;
      exit(EXIT_FAILURE);
    }

  numberOfFeatures = numberOfFeaturesExtracted;

  if(isTraining == false)
    loadLearnedAlgorithm();
}

/**************************************/
//The function to train the feature reduction algorithm without training labels <**Overloaded function**>
void FeatureReduction::trainFeatureReducer(Mat & trainingFeatures)
{
  if(reductionType == "PCA")
    {
      PrincipalComponentAnalysisTraining(trainingFeatures);
    }
  else
    {
      cerr << "\n\nInvalid call to the function trainFeatureReducer in class FeatureReduction.\nExiting...\n" << endl;
      exit(EXIT_FAILURE);
    }

  return;
}

/**************************************/
//The function to train the feature reduction algorithm with training labels <**Overloaded function**>
void FeatureReduction::trainFeatureReducer(Mat & trainingFeatures, Mat & trainingLabels)
{
  if(reductionType == "LDA")
    {
      LinearDiscriminantAnalysisTraining(trainingFeatures, trainingLabels);
    }
  else if(reductionType == "PCA->LDA")
    {
      savedNumberOfFeatures = numberOfFeatures;
      numberOfFeatures = static_cast<int>(numberOfFeatures * PCA_TO_LDA_RATIO);
      PrincipalComponentAnalysisTraining(trainingFeatures);
      reductionType = "PCA";
      reduceFeatures(trainingFeatures, reducedFeatures);
      reductionType = "PCA->LDA";
      numberOfFeatures = savedNumberOfFeatures;
      LinearDiscriminantAnalysisTraining(reducedFeatures, trainingLabels);
    }
    else
      {
        cerr << "\n\nInvalid call to the function trainFeatureReducer in class FeatureReduction.\nExiting...\n" << endl;
        exit(EXIT_FAILURE);
      }

  return;
}

/**************************************/
//The function to get te reduced feature vector
void FeatureReduction::reduceFeatures(Mat & inputFeatureVector, Mat & outputFeatureVector)
{
  if(reductionType == "PCA")
    {
      outputFeatureVector = PCA_compressor.project(inputFeatureVector);
      outputFeatureVector.convertTo(outputFeatureVector, CV_32F);
    }
  else if(reductionType == "LDA")
    {
      outputFeatureVector = LDA_compressor.project(inputFeatureVector);
      outputFeatureVector.convertTo(outputFeatureVector, CV_32F);
    }
  else if(reductionType == "PCA->LDA")
    {
      interimFeatureVector = PCA_compressor.project(inputFeatureVector);
      outputFeatureVector = LDA_compressor.project(interimFeatureVector);
      outputFeatureVector.convertTo(outputFeatureVector, CV_32F);
    }

  return;
}

/**************************************/
//Principal Component Analysis's training function
void FeatureReduction::PrincipalComponentAnalysisTraining(Mat & trainingFeatures)
{
  //training the PCA method
  PCA_compressor = PCA(trainingFeatures, Mat(), CV_PCA_DATA_AS_ROW, numberOfFeatures);

  //saving the trained PCA
  filename.str(string());
  filename << FEATURE_REDUCTION_SOURCE_FOLDER << "/" << TRAINING_SUBDIRECTORY_NAME[SIMULATION_WORLD_ENABLE] << "/PCA/PCA.yml";
  saveFilename = FileStorage(filename.str(), FileStorage::WRITE);
  PCA_compressor.write(saveFilename);

  return;
}

/**************************************/
//Linear Discriminant Analysis's training function
void FeatureReduction::LinearDiscriminantAnalysisTraining(Mat & trainingFeatures, Mat & trainingLabels)
{
  //training the LDA method
  LDA_compressor = LDA(trainingFeatures, trainingLabels, numberOfFeatures);

  //saving the trained data
  filename.str(string());
  filename << FEATURE_REDUCTION_SOURCE_FOLDER << "/" << TRAINING_SUBDIRECTORY_NAME[SIMULATION_WORLD_ENABLE] << "/LDA/LDA.yml";
  saveFilename = FileStorage(filename.str(), FileStorage::WRITE);
  LDA_compressor.save(saveFilename);

  return;
}

/**************************************/
//Function to load trained feature reduction algorithms from files
void FeatureReduction::loadLearnedAlgorithm()
{
  if(reductionType == "PCA")
    {
      filename.str(string());
      filename << FEATURE_REDUCTION_SOURCE_FOLDER << "/" << TRAINING_SUBDIRECTORY_NAME[SIMULATION_WORLD_ENABLE] << "/PCA/PCA.yml";
      loadFilename = FileStorage(filename.str(), FileStorage::READ);

      PCA_compressor.read(loadFilename.root());
    }
  else if(reductionType == "LDA")
    {
      filename.str(string());
      filename << FEATURE_REDUCTION_SOURCE_FOLDER << "/" << TRAINING_SUBDIRECTORY_NAME[SIMULATION_WORLD_ENABLE] << "/LDA/LDA.yml";
      loadFilename = FileStorage(filename.str(), FileStorage::READ);

      LDA_compressor.load(loadFilename);
    }
  else if(reductionType == "PCA->LDA")
    {
      //load PCA
      filename.str(string());
      filename << FEATURE_REDUCTION_SOURCE_FOLDER << "/" << TRAINING_SUBDIRECTORY_NAME[SIMULATION_WORLD_ENABLE] << "/PCA/PCA.yml";
      loadFilename = FileStorage(filename.str(), FileStorage::READ);

      PCA_compressor.read(loadFilename.root());

      //load LDA
      filename.str(string());
      filename << FEATURE_REDUCTION_SOURCE_FOLDER << "/" << TRAINING_SUBDIRECTORY_NAME[SIMULATION_WORLD_ENABLE] << "/LDA/LDA.yml";
      loadFilename = FileStorage(filename.str(), FileStorage::READ);

      LDA_compressor.load(loadFilename);
    }

  return;
}

