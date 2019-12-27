/*
 * MyClassifier.cpp
 *
 *  Created on: Mar 28, 2017
 *      Author: pourya
 */

//Header file
#include "Classification.h"

//*****************************************
//Constructor
Classification::Classification(classifierTypes classification_method)
{
  //Set the probability of category "all objects" to zero (applicable in Dempster-Shafer decision fusion)
  allObjectsProbability = 0;

  classificationMethod = classification_method;

  //Initializing variables
  classificationLabel = "";

  //Loading trained classifier
  loadLearningMemory();

  //Creating a feature reduction object
  if (ENABLE_FEATURE_REDUCTION)
    HOG_featureReduction = FeatureReduction(FEATURE_REDUCTION_METHOD, REDUCED_HOG_NUMBER_OF_FEATURES, false);

  return;
}

//*****************************************
//The function to classify that returns probabilities of each class
Mat & Classification::Classify(Mat & classification_input)
{
  //Calling the appropriate classifier
  if (classificationMethod == classifierTypes::SVM)
    {
      estimatedScores = SupportVectorMachine(classification_input);
    }

  //Rescale the object probabilities to sum to 1
  if (DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer)
    estimatedScores = estimatedScores * (1.0f / sum(estimatedScores).val[0]);

  //Returning the per class probabilities
  return estimatedScores;
}

//*****************************************
//The function to get the probability of category "Unknown Object"
double & Classification::getAllObjectsProbability()
{
  //Return the requested probability
  return allObjectsProbability;
}

//*****************************************
//Support Machine Classifier (SVM) classifier
Mat & Classification::SupportVectorMachine(Mat & input_image)
{
  //Extracting features
  features.extractFeatures(input_image, colorHistogram, HOG);

  if (ENABLE_FEATURE_REDUCTION)
    {
      //feature reduction
      HOG_featureReduction.reduceFeatures(HOG, reducedHOG);

      //Simeple concatination of features
      featureVector = Mat();
      hconcat(colorHistogram, reducedHOG, featureVector);
    }
  else
    {
      //Simeple concatination of features
      featureVector = Mat();
      hconcat(colorHistogram, HOG, featureVector);
    }

  /*Call the Scikit-learn's SVM classifier*/

  //Import the python module
  ClassifierModule_BoostObject = bp::import(PYTHON_CLASSIFIER_MODULE_NAME);

  //Converting mat to numpy ndarray
  np::ndarray featureVector_ndarrayPy = floatMatNdarrayConvertor.mat_to_ndarray_2d(featureVector);

  //Preparing the arguments to the python method
  scikitPythonArgs = bp::list();
  scikitPythonArgs.append(featureVector_ndarrayPy);

  try
  {
      //Loading the python classifier class (Not here! It is loaded in loadLearningMemory member function)
      //FeatureClassifierClass_BoostObject = ClassifierModule_BoostObject.attr(PYTHON_FEATURE_CLASSIFIER_CLASS)();

      //Calling the scikit-learn SVM classifier method
      classificationResult_Object = featureClassifierClass_BoostObject.attr(PYTHON_SVM_CLASSIFIER_METHOD)(* bp::tuple(scikitPythonArgs));

      //Converting Boost.Python object to C++ types
      np::ndarray estimatedScores_ndarrayd = np::from_object(classificationResult_Object[0]);
      estimatedScores = doubleMatNdarrayConvertor.ndarray_to_mat_2d(estimatedScores_ndarrayd); //saving the detected class probabilities

      //If Dempster-Shafer fusion is enabled, separate the last probability, "all objects" from the rest
      if (DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer)
	{
	  allObjectsProbability = estimatedScores.at<double>(0, estimatedScores.cols - 1);
	  estimatedScores = estimatedScores(Rect(0, 0, estimatedScores.cols - 1, 1));
	}
  }
  catch (const bp::error_already_set&)
  {
      PyErr_Print();
      exit(EXIT_FAILURE);
  }

  return estimatedScores;
}

//*****************************************
//The function to load the learning memory into the classifier
void Classification::loadLearningMemory()
{
  //Set the classification source directory
  classificationSourceDirectory = CLASSIFIER_SOURCE_FOLDER + string("/") + TRAINING_SUBDIRECTORY_NAME[SIMULATION_WORLD_ENABLE];

  if(classificationMethod == classifierTypes::SVM)
    {
      classifierFolderName = "SVM";

      //Loading label equivalencies
      ObjectListReader(objectNamesList);

      //Import the python module
      ClassifierModule_BoostObject = bp::import(PYTHON_CLASSIFIER_MODULE_NAME);

      //Preparing the arguments to the python method
      PythonArgs = bp::list();
      PythonArgs.append(PYTHON_SVM_CLASSIFIER_METHOD);
      PythonArgs.append(classificationSourceDirectory);
      PythonArgs.append(classifierFolderName);
      PythonArgs.append(bool(DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer));

      try
      {
	  //Loading the python classifier class
	  featureClassifierClass_BoostObject = ClassifierModule_BoostObject.attr(PYTHON_FEATURE_CLASSIFIER_CLASS)();

	  //Calling the method for loading training data
	  featureClassifierClass_BoostObject.attr(PYTHON_CLASSIFIER_LOADING_METHOD)(* bp::tuple(PythonArgs));
      }
      catch (const bp::error_already_set&)
      {
	  PyErr_Print();
	  exit(EXIT_FAILURE);
      }
    }
  else
    {
      fprintf(stderr, "The classifier is not recognized.\nExiting...\n\n");
      exit(EXIT_FAILURE);
    }

  return;
}

//*****************************************
//The function to find the maximum score and winner label out of a class probability vector
void Classification::FindWinnerClass(Mat & inputClassProbabilities, string & outputClassificationLabel, double & outputEstimatedLabelScore, int * output_winnerIndex)
{
  //Finding the maximum and index of the maximum score
  minMaxLoc(inputClassProbabilities, & minMat, & maxMat, & minLocation, & maxLocation);

  //Saving the maximum location in the output variable
  outputClassificationLabel = objectNamesList[static_cast<int>(maxLocation.x)];

  //Saving the maximum value in the output variable
  outputEstimatedLabelScore = maxMat;

  //Saving the winner class's index, if enabled
  if(output_winnerIndex != NULL)
    * output_winnerIndex = static_cast<int>(maxLocation.x);

  return;
}

//*****************************************
//The function to read the list of objects from a file. Returns the number of objects
int ObjectListReader(vector<string> & output_objectNamesList)
{
  //Declarations
  stringstream trainingFileName;
  FileStorage labelsBook;
  int objectCounter;
  char objectNumber[10];
  string objectName, classifierFolderName;

  //Deciding on the folder that contains the object list based on the specified classifier
  if(CLASSIFIER_TYPE == classifierTypes::SVM)
    classifierFolderName = "SVM";
  else
    {
      fprintf(stderr, "The classifier is not recognized.\nExiting...\n\n");
      exit(EXIT_FAILURE);
    }

  //Generating the file address for the objects list
  trainingFileName.str(std::string());
  trainingFileName << CLASSIFIER_SOURCE_FOLDER << "/" << TRAINING_SUBDIRECTORY_NAME[SIMULATION_WORLD_ENABLE] << "/" << classifierFolderName << "/Labels-Book.yml";

  //Opening the objects list file
  labelsBook.open(trainingFileName.str(), cv::FileStorage::READ);

  if (! labelsBook.isOpened())
    {
      cerr << "Cannot find the file " << trainingFileName.str() << ". Exiting...\n" << endl;
      exit(EXIT_FAILURE);
    }

  //Reading the object labels and storing them
  objectCounter = 0;
  while(true)
    {
      sprintf(objectNumber, "Number %d", objectCounter);
      if(! labelsBook[objectNumber].isNone())
	labelsBook[objectNumber] >> objectName;
      else
	break;
      output_objectNamesList.push_back(objectName);

      objectCounter++;
  }

  //Closing the opened objects list file
  labelsBook.release();

  //Check if a per class threshold is being used, there are enough probabilities for all the classes
  if(DIFFERENT_PROBABILITY_THRESHOLD_PER_CLASS)
    {
      if(OBJECT_PROBABILITY_THRESHOLD_PER_CLASS.size() == 0) //Raise error and exit if the probability threshold array is empty
	{
	  fprintf(stderr, "\n\nThe array for object probability thresholds per class is empty.\nExiting...\n\n");
	  exit(EXIT_FAILURE);
	}
      if(output_objectNamesList.size() > OBJECT_PROBABILITY_THRESHOLD_PER_CLASS.size()) //Raise error and exit if the number of the probability thresholds is less than the number of classes
	{
	  fprintf(stderr, "The array for object probability thresholds per class has not enough probabilities for all the classes.\nExiting...\n\n");
	  exit(EXIT_FAILURE);
	}
      else if(output_objectNamesList.size() < OBJECT_PROBABILITY_THRESHOLD_PER_CLASS.size()) //Warn that the number of probability thresholds is more than the number of classes
	printf("Warning: The array for object probability thresholds per class has extra probabilities than the number of all the classes.\n");
    }

  return objectCounter;
}


