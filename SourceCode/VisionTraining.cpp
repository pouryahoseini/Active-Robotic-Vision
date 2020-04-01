/*
 * VisionTraining.cpp
 *
 *  Created on: Mar 27, 2017
 *      Author: pourya
 */

//Header file
#include "VisionTraining.h"


//*****************************************
//Constructor
VisionTraining::VisionTraining(classifierTypes training_method)
{
  //set training sample skip to false (for Dempster-Shafer decision fusion)
  trainingSampleSkip = false;

  //Save the training method
  trainingMethod = training_method;

  //Set initial values for variables
  emptyImage = true;
  continueTraining = true;
  sampleCounter = 0;
  folderCounter = 0;
  fileCounter = 1;
  currentFolderCounter = 0;

  //Read the list of object classes
  objectListReader();

  //Create feature reduction object, if enabled
  if (ENABLE_FEATURE_REDUCTION)
    HOG_featureReduction = FeatureReduction(FEATURE_REDUCTION_METHOD, REDUCED_HOG_NUMBER_OF_FEATURES, true);

  return;
}

//*****************************************
//The function to run the training
void VisionTraining::Train()
{
  //Call the appropriate classifier to train
  if (trainingMethod == classifierTypes::SVM)
    {
      printf("\nTraining by the Support Vector Machine learning.\n");
      classifierFolderName = "SVM";
      SupportVectorMachine();
    }
  else
    {
      fprintf(stderr, "\nError: The learning method is not identified.\nTraining not finished.\n\n");
      exit(EXIT_FAILURE);
    }

  return;
}

//*****************************************
//SVM Learning
void VisionTraining::SupportVectorMachine()
{
  //Read the list of objects
  ObjectListWriter();

  //Fectch the first image
  continueTraining = getNextTrainingSample(trainingImage);

  /* Construct the vector of samples and their features */
  while(continueTraining == true)
    {
      //Extract the current sample image's features
      features.extractFeatures(trainingImage, colorHistogram, HOG);

      //Save the characteristics of the current image
      sampleColorHistMat.push_back(colorHistogram);
      sampleHOGMat.push_back(HOG);
      sampleLabelCodes.push_back(currentFolderCounter);
      sampleLabels.push_back(objects[currentFolderCounter]);

      //In the case of DST fusion, add the current training sample with the label "Unknown Object" once a while
      if (DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer)
	if (trainingSampleSkip != true)
	  {
	    sampleColorHistMat.push_back(colorHistogram);
	    sampleHOGMat.push_back(HOG);
	    sampleLabelCodes.push_back(numberOfObjects);
	    sampleLabels.push_back(DEMPSTER_SHAFER_ALL_OBJECTS_CATEGORY_NAME);

	    //Increament number of training samples read so far
	    sampleCounter++;

	    //The next training sample won't be read
	    trainingSampleSkip = true;
	  }
	else
	  //The next training sample will be read
	  trainingSampleSkip = false;

      //Increament number of training samples read so far
      sampleCounter++;

      //Fetch another training image
      continueTraining = getNextTrainingSample(trainingImage);
    }

  sampleLabelCodes.reshape(sampleCounter, 1);
  sampleLabelCodes.convertTo(sampleLabelCodes, CV_32SC1);

  if (ENABLE_FEATURE_REDUCTION)
    {
      //Train the feature reduction algorithm
      if(FEATURE_REDUCTION_METHOD == "PCA")
	HOG_featureReduction.trainFeatureReducer(sampleHOGMat);
      else if( (FEATURE_REDUCTION_METHOD == "LDA") || (FEATURE_REDUCTION_METHOD == "PCA->LDA") )
	HOG_featureReduction.trainFeatureReducer(sampleHOGMat, sampleLabelCodes);

      //Performing feature reduction on the training data
      HOG_featureReduction.reduceFeatures(sampleHOGMat, reducedHOG);

      //Concatinating feature vectors
      hconcat(sampleColorHistMat, reducedHOG, sampleFeatureMatrix);
    }
  else
    {
      //Concatinating feature vectors
      hconcat(sampleColorHistMat, sampleHOGMat, sampleFeatureMatrix);
    }

  //Import the python module
  ClassifierModule_BoostObject = bp::import(PYTHON_TRAINING_MODULE_NAME);

  //Converting mat to numpy ndarray
  np::ndarray sampleFeatureMatrix_ndarrayPy = floatMatNdarrayConvertor.mat_to_ndarray_2d(sampleFeatureMatrix);
  np::ndarray sampleLabelCodes_ndarrayPy = intMatNdarrayConvertor.mat_to_ndarray_2d(sampleLabelCodes);

  //Set the classification source directory
  classificationSourceDirectory = CLASSIFIER_SOURCE_FOLDER + string("/") + TRAINING_SUBDIRECTORY_NAME[SIMULATION_WORLD_ENABLE];

  //Preparing the arguments to the python method
  scikitPythonArgs = bp::list();
  scikitPythonArgs.append(sampleFeatureMatrix_ndarrayPy);
  scikitPythonArgs.append(sampleLabelCodes_ndarrayPy);
  scikitPythonArgs.append(classificationSourceDirectory);
  scikitPythonArgs.append(classifierFolderName);
  scikitPythonArgs.append(SCIKIT_LEARN_SVM_KERNEL);
  scikitPythonArgs.append(bool(DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer));

  try
  {
      //Loading the python classifier class
      featureClassifierClass_BoostObject = ClassifierModule_BoostObject.attr(PYTHON_FEATURE_CLASSIFIER_TRAINING_CLASS)();

      //Calling the scikit-learn SVM classifier method
      featureClassifierClass_BoostObject.attr(PYTHON_SVM_CLASSIFIER_METHOD)(* bp::tuple(scikitPythonArgs));
  }
  catch (const bp::error_already_set&)
  {
      PyErr_Print();
      exit(EXIT_FAILURE);
  }

  return;
}

//*****************************************
//Function to get the next training sample, returns true if training should continue
bool VisionTraining::getNextTrainingSample(Mat & output_trainnigSample)
{
  //Get a training image
  if(! DATA_AUGMENTATION || ((! DATA_AUGMENTATION_BACKGROUND_CLASS) && (objects[folderCounter] == BACKGROUND_CLASS_NAME))) //Get a training image if data augmentation is inactive or the objects class is Background
    continueTraining = fetchTrainingImage(output_trainnigSample); //Get an original training sample
  else if(fetchOriginalSample) //Get a training image before generating augmented artificial samples
    {
      //Get an original training sample
      continueTraining = fetchTrainingImage(output_trainnigSample);

      //Let artificial samples be created from this image in the next calls of this function
      fetchOriginalSample = false;
      copiedOriginalSample = output_trainnigSample.clone();
    }
  else //Generate augmented samples
    {
      //Cropping and zooming in the image
      while(cropCounter < 5)
	{
	  output_trainnigSample = sampleAugmentation.cropSample(copiedOriginalSample, cropCounter);
	  cropCounter++;
	  return true;
	}

      //Blur the image with the Gaussian filtering
      while(blurringCounter < 3)
	{
	  output_trainnigSample = sampleAugmentation.blurSample(copiedOriginalSample, blurringCounter);
	  blurringCounter++;
	  return true;
	}

      while(brightnessCounter < 6)
	{
	  output_trainnigSample = sampleAugmentation.changeBrightness(copiedOriginalSample, brightnessCounter);
	  brightnessCounter++;
	  return true;
	}

      //Adding Gaussian noise to the saturation and intensity channels
      while(noiseAddCounter < 3)
	{
	  output_trainnigSample = sampleAugmentation.addNoise(copiedOriginalSample, noiseAddCounter);
	  noiseAddCounter++;
	  return true;
	}

      //Geometric transformations
      while(geoTransformCounter < 5)
	{
	  output_trainnigSample = sampleAugmentation.transformGeometrically(copiedOriginalSample, geoTransformCounter);
	  geoTransformCounter++;

	  //Check for the last round to reset variables to get a new original training sample next time
	  if(geoTransformCounter == 5)
	    {
	      //Reset the counters
	      geoTransformCounter = 0;
	      noiseAddCounter = 1;
	      brightnessCounter = 0;
	      blurringCounter = 0;
	      cropCounter = 0;

	      //Fetch another original training sample next time
	      fetchOriginalSample = true;
	    }

	  return true;
	}
    }

  return continueTraining;
}

//*****************************************
//The function to sweep the full size training files, returns true if training should continue
bool VisionTraining::fetchTrainingImage(Mat & output_trainingImage)
{
  do
    {
      if (folderCounter < numberOfObjects)
	{
	  emptyImage = false;

      	  fileName.str(std::string());
      	  fileName << TRAINING_SOURCE_FOLDER << "/" << TRAINING_SUBDIRECTORY_NAME[SIMULATION_WORLD_ENABLE] << "/" << objects[folderCounter] << "/" << fileCounter << TRAINING_OBJECTS_EXTENSION;

      	  output_trainingImage = imread(fileName.str(), CV_LOAD_IMAGE_COLOR);

      	  currentFolderCounter = folderCounter;

      	  if (! output_trainingImage.data)
      	    {
      	      emptyImage = true;
      	      fileCounter = 0;
      	      ++folderCounter;
      	    }

      	  ++fileCounter;
	}
      else
        return false; //finished the training
    } while((emptyImage == true) && (continueTraining == true));

  return true;
}

//*****************************************
//The function to write the list of objects into a file
void VisionTraining::ObjectListWriter()
{
  trainingFileName.str(std::string());
  trainingFileName << CLASSIFIER_SOURCE_FOLDER << "/" << TRAINING_SUBDIRECTORY_NAME[SIMULATION_WORLD_ENABLE] << "/" << classifierFolderName << "/Labels-Book.yml";

  labelsBook.open(trainingFileName.str(), cv::FileStorage::WRITE);

  for(int i = 0; i < numberOfObjects; i++)
    {
      sprintf(objectNumber, "Number %d", i);
      labelsBook << objectNumber << objects[i];
    }

  labelsBook.release();

  return;
}

//*****************************************
//The function to read the list of objects from a text file
void VisionTraining::objectListReader()
{
  //Specify exception in opening the file
  objectsList.exceptions(ifstream::badbit);

  //Clear the list of objects
  objects.clear();

  //Specify the file name
  fileName.str(std::string());
  fileName << TRAINING_SOURCE_FOLDER << "/" << TRAINING_SUBDIRECTORY_NAME[SIMULATION_WORLD_ENABLE] << "/" << TRAINING_OBJECTS_LIST;

  //Open the file
  try
  {
      objectsList.open(fileName.str().c_str());
  }
  catch(const ifstream::failure& e)
  {
      cout << "Error in opening the " << TRAINING_SOURCE_FOLDER << "/" << TRAINING_SUBDIRECTORY_NAME[SIMULATION_WORLD_ENABLE] << "/" << TRAINING_OBJECTS_LIST << " file." << endl;
      exit(EXIT_FAILURE);
  }

  //Set number of objects to zero initially
  numberOfObjects = 0;

  //Fetch all the listed object names
  while(objectsList >> objectName)
    {
      objects.push_back(objectName);
      numberOfObjects++;
    }

  //Close the file
  objectsList.close();

  return;
}

/****************************************************  artificialSampleGeneration  **********************************************************/
//*****************************************
//The function to add Gaussian noise to the input image
Mat & dataAugmentation::addNoise(Mat & input_image, int HSVChannel, float standardDeviation)
{
  //Check the input channel number to be in the bound
  if((HSVChannel > 2) || (HSVChannel < 0))
    {
      cerr << "\nThe HSV channel number is not between 0 and 2\nExiting...\n" << endl;
      exit(EXIT_FAILURE);
    }

  //Extract the HSV channels
  cvtColor(input_image, processedImage, CV_BGR2HSV);

  //Split channels
  split(processedImage, separatedChannels);

  //Converting the channel image to float to prevent overflow and underflow (values below zero or above 255)
  separatedChannels[HSVChannel].convertTo(floatImage, CV_32FC1, 1.0/255.0);

  //Preserving the original image's dynamic range
  minMaxLoc(floatImage, & minPixelValue, & maxPixelValue);

  //Add Guassian noise
  noise = Mat(floatImage.size(), CV_32FC1);
  randn(noise , 0, standardDeviation);
  floatImage = floatImage + noise;

  //Normalize the image to ensure the pixel values would not go more than the maximum possible value
  normalize(floatImage, floatImage, minPixelValue, maxPixelValue, NORM_MINMAX);

  //Convert the image back to uint type
  floatImage.convertTo(separatedChannels[HSVChannel], separatedChannels[HSVChannel].type(), 255.0);

  //Merge channels
  merge(separatedChannels, 3, processedImage);

  //Conver the image to BGR domain
  cvtColor(processedImage, processedImage, CV_HSV2BGR);

  return processedImage;
}

//*****************************************
//The function to blur images with directional or omni-directional Gaussian filter
Mat & dataAugmentation::blurSample(Mat & input_image, int blurMode)
{
  //Decide on the blurring mode
  blurMode %= 3;
  switch(blurMode)
  {
    case 0: //Omni-directional blurring
      GaussianBlur(input_image, processedImage, Size(7, 7), 2);
      break;
    case 1: //Horizontal blurring
      GaussianBlur(input_image, processedImage, Size(7, 1), 2);
      break;
    case 2: //Vertical blurring
      GaussianBlur(input_image, processedImage, Size(1, 7), 2);
      break;
  }

  return processedImage;
}

//*****************************************
//The function to crop and zoom in the training images
Mat & dataAugmentation::cropSample(Mat & input_image, int cropMode)
{
  //Save the input image's size
  imageSize = input_image.size();

  //Decide on the type (or side) of the crop
  cropMode %= 5;
  switch(cropMode)
  {
    case 0: //Zoom by a factor of 2
      resize(input_image(Rect(imageSize.width / 4.0, imageSize.height / 4.0, imageSize.width / 2.0, imageSize.height / 2.0)), processedImage, imageSize);
      break;
    case 1: //Crop one third from top
      resize(input_image(Rect(0, imageSize.height / 3.0, imageSize.width, 2.0 * imageSize.height / 3.0)), processedImage, imageSize);
      break;
    case 2: //Crop one third from bottom
      resize(input_image(Rect(0, 0, imageSize.width, 2.0 * imageSize.height / 3.0)), processedImage, imageSize);
      break;
    case 3: //Crop one third from left
      resize(input_image(Rect(imageSize.width / 3.0, 0, 2.0 * imageSize.width / 3.0, imageSize.height)), processedImage, imageSize);
      break;
    case 4: //Crop one third from right
      resize(input_image(Rect(0, 0, 2.0 * imageSize.width / 3.0, imageSize.height)), processedImage, imageSize);
      break;
  }

  return processedImage;
}

//*****************************************
//The function to changes brightness of a sample image
Mat & dataAugmentation::changeBrightness(Mat & input_image, int brightnessMode)
{
  //Convert the input image to the HSV color domain
  cvtColor(input_image, HSVImage, CV_BGR2HSV);

  //Split the channels
  split(HSVImage, HSVChannels);

  //Decide on the brightness mode
  brightnessMode %= 6;
  switch(brightnessMode)
  {
    case 0:
      HSVChannels[2] = HSVChannels[2] + 40;
      break;
    case 1:
      HSVChannels[2] = HSVChannels[2] + 80;
      break;
    case 2:
      HSVChannels[2] = HSVChannels[2] + 120;
      break;
    case 3:
      HSVChannels[2] = HSVChannels[2] - 40;
      break;
    case 4:
      HSVChannels[2] = HSVChannels[2] - 80;
      break;
    case 5:
      HSVChannels[2] = HSVChannels[2] - 120;
      break;
  }

  //Merge the channels
  merge(HSVChannels, 3, processedImage);

  //Convert the output image to the BGR color domain
  cvtColor(processedImage, processedImage, CV_HSV2BGR);

  return processedImage;
}

//*****************************************
//The function to transform images geometrically
Mat & dataAugmentation::transformGeometrically(Mat & input_image, int transformType)
{
  //Decide on the type of the geometric transformation
  transformType %= 5;
  switch(transformType)
  {
    case 0:
      rotate(input_image, processedImage, ROTATE_90_CLOCKWISE); //90 degrees rotation
      break;
    case 1:
      rotate(input_image, processedImage, ROTATE_180); //180 degrees rotation
      break;
    case 2:
      rotate(input_image, processedImage, ROTATE_90_COUNTERCLOCKWISE); //270 degrees rotation
      break;
    case 3:
      flip(input_image, processedImage, 0); //Vertical flip
      break;
    case 4:
      flip(input_image, processedImage, 1); //Horizontal flip
      break;
  }

  return processedImage;
}

