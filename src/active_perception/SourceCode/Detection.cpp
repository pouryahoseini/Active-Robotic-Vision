/*
 * Detector.cpp
 *
 *  Created on: Jul 24, 2018
 *      Author: pourya
 */

//Header
#include "Detection.h"

//*****************************************
//Constructor
Detection::Detection(detectorTypes detection_method, classifierTypes classification_method, int inputImage_Width, int inputImage_Height)
{
  //Save the detection and classification methods
  detectionMethod = detection_method;
  classificationMethod = classification_method;

  //Create a Mixture of Gaussians background subtractor if it is enabled
  if(detectionMethod == detectorTypes::Background_Subtraction)
    MoGBackSub = createBackgroundSubtractorMOG2(MoG_history, MoG_variableThreshold, false).dynamicCast<BackgroundSubtractor>();

  //Create a morphological structuring element if a detection method with foreground processing is being used
  if((detectionMethod == detectorTypes::Background_Subtraction) || (detectionMethod == detectorTypes::Intensity_Thresholding))
    structuringElement = getStructuringElement(MORPH_RECT, Size(static_cast<int>(STRUCTURING_ELEMENT_TO_IMAGE_RATIO * min(inputImage_Width, inputImage_Height)), static_cast<int>(STRUCTURING_ELEMENT_TO_IMAGE_RATIO * min(inputImage_Width, inputImage_Height))));

  //Set the intensity threshold (for intensity threshold based segmentation)
  intensityThreshold = INTENSITY_THRESHOLD;

  //Decide on which detection function should be called upon calling the function "Detect"
  if(detectionMethod == detectorTypes::Background_Subtraction) //Background subtraction detection
    detectionFunction_ptr = & Detection::BackgroundSubtraction;
  else if(detectionMethod == detectorTypes::Intensity_Thresholding) //Intensity threshold-based segmentation detection
    detectionFunction_ptr = & Detection::IntensityThresholding;
  else if(detectionMethod == detectorTypes::Sliding_Window)
    detectionFunction_ptr = & Detection::SlidingWindow;

  //Set the input image boundary discard ratio
  if(REJECT_INPUT_IMAGE_BOUNDARIES)
    discardRatio = LENGTH_WIDTH_DISCARD_RATIO;
  else
    discardRatio = 0;
}

//*****************************************
//Function to detect objects in an image
vector<objectInfo> Detection::Detect(Mat & input_image, int cameraNumber, Mat * depthImage)
{
  //Check if the depth image should be used, it is not empty (the depthImage argument is optional)
  if(depthImage != nullptr)
    useDepth = true;
  else
    useDepth = false;

  //Clear the vector of object info
  detectedObjectsInfo.clear();

  //Call the appropriate detection function
  detectedObjectsInfo = (this->* detectionFunction_ptr)(input_image, cameraNumber, depthImage);

  //Initialize all the object trackers
  trackingManager.createAndInitilizeTrackers(input_image, detectedObjectsInfo);

  return detectedObjectsInfo;
}

//*****************************************
//Function to detect objects based on the sliding window technique
vector<objectInfo> & Detection::SlidingWindow(Mat & input_image, int & cameraNumber, Mat * depthImage)
{

  //Blacken part of the image that have out of range depths
  if(REJECT_ABNORMAL_DISTANCES && useDepth)
    {
      //Blacken out of range areas in the BGR image
      depthMask1 = (* depthImage) > MIN_DEPTH;
      depthMask2 = (* depthImage) < MAX_DEPTH;
      cv::bitwise_and(depthMask1, depthMask2, depthMask);
      pyramidImage = Mat::zeros(input_image.size(), input_image.type());
      input_image.copyTo(pyramidImage);

      //Construct the depth image's pyramid
      (* depthImage).copyTo(depthPyramid);
    }
  else
    {
      //Copy the input image to another matrix to prevent modifying it
      input_image.copyTo(pyramidImage);
    }

  //Go up (larger) in the image pyramid initially
  for(pyrupCounter = 0; pyrupCounter < SLIDING_WINDOW_INITIAL_MAGNIFICATION; pyrupCounter++)
    {
      pyrUp(pyramidImage, pyramidImage);
      pyrUp(depthPyramid, depthPyramid);
    }

  //Construct a gray-scale image pyramid to later reject areas based on their intensities, if enabled
  if(REJECT_INTENSITY_BASED)
    cvtColor(pyramidImage, intensityPyramid, CV_BGR2GRAY);

  //Determine the number of sliding window shapes
  numberOfShapes = 2 * NUMBER_OF_WINDOW_SLIMMINGS_EACH_DIMENSION + 1;

  //Clear the vectors of bounding boxes and their scores
  allProbabilities.clear();
  allBoundingBoxes.clear();
  allWinnerProbabilities.clear();
  allWinnerLabels.clear();
  if (DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer)
    allUnknownProbabilities.clear();

  //Repeat for different image sizes in the image pyramid
  for(pyramidLevelCounter = 0; pyramidLevelCounter < IMAGE_PYRAMID_NUMBER_OF_LAYERS; pyramidLevelCounter++)
    {
      //Setting initial window height and width
      windowHeight = SLIDING_WINDOW_SIZE.height;
      windowWidth = SLIDING_WINDOW_SIZE.width;

      //Check the size of the image compared to the sliding window size
      if((pyramidImage.size().width * (1 - discardRatio) < SLIDING_WINDOW_SIZE.width) || (pyramidImage.size().height * (1 - discardRatio) < SLIDING_WINDOW_SIZE.height))
	break;

      //Repeat for different window aspect ratios
      for(shapeCounter = 0; shapeCounter < numberOfShapes; shapeCounter++)
	{
	  //Decide on the next shape (aspect ratio) of the sliding window
	  if(shapeCounter == 0)
	    windowSize = SLIDING_WINDOW_SIZE;
	  else if(shapeCounter <= NUMBER_OF_WINDOW_SLIMMINGS_EACH_DIMENSION)
	    {
	      windowHeight /= 2;
	      windowSize = Size(SLIDING_WINDOW_SIZE.width, windowHeight);
	    }
	  else
	    {
	      windowWidth /= 2;
	      windowSize = Size(windowWidth, SLIDING_WINDOW_SIZE.height);
	    }

	  //Repeat for different pixel locations
	  for(pixelWidth = 0 + (discardRatio / 2.0) * pyramidImage.size().width; pixelWidth <= (pyramidImage.size().width * (1 - (discardRatio / 2.0)) - windowSize.width); pixelWidth += SLIDING_WINDOW_STEP_SIZE)
	    for(pixelHeight = 0 + (discardRatio / 2.0) * pyramidImage.size().height; pixelHeight <= (pyramidImage.size().height * (1 - (discardRatio / 2.0)) - windowSize.height); pixelHeight += SLIDING_WINDOW_STEP_SIZE)
	      {
		//In the case of depth filtering, prevent detection if the current location's depth is out of range
		if(REJECT_ABNORMAL_DISTANCES && useDepth)
		  {
		    pixelDepth = static_cast<int>(depthPyramid.at<uchar>(pixelHeight, pixelWidth));
		    if((pixelDepth > MAX_DEPTH) || (pixelDepth < MIN_DEPTH))
		      continue;
		  }

		//Reject areas based on their intensities, if enabled
		if(REJECT_INTENSITY_BASED && (static_cast<int>(intensityPyramid.at<uchar>(pixelHeight, pixelWidth)) < intensityThreshold))
		  continue;

		//Crop the image to contain only the current object
		currentSlidingWindow = Rect(Point(pixelWidth, pixelHeight), windowSize);
		objectImage = pyramidImage(currentSlidingWindow);

		//Classify object
		objectProbabilities = Classify(objectImage);

		//Check if the objectness in the current sliding window (by comparing the maximum probability to the second max)
		FindWinnerClass(objectProbabilities, winnerObjectLabel, winnerProbability, & winnerIndex);

		if((winnerObjectLabel != BACKGROUND_CLASS_NAME) && (DIFFERENT_PROBABILITY_THRESHOLD_PER_CLASS || (winnerProbability > GLOBAL_OBJECT_PROBABILITY_THRESHOLD)) && ((! DIFFERENT_PROBABILITY_THRESHOLD_PER_CLASS) || (winnerProbability > OBJECT_PROBABILITY_THRESHOLD_PER_CLASS[winnerIndex])))
		  {
		    //Convert the bounding box dimensions and location for the original size image
		    currentSlidingWindow.x *= (input_image.size().width / ((float) pyramidImage.size().width));
		    currentSlidingWindow.y *= (input_image.size().height / ((float) pyramidImage.size().height));
		    currentSlidingWindow.width *= (input_image.size().width / ((float) pyramidImage.size().width));
		    currentSlidingWindow.height *= (input_image.size().height / ((float) pyramidImage.size().height));

		    //Save the current bounding box for now to be subjected to non-maximum suppression later
		    allProbabilities.push_back(objectProbabilities);
		    allBoundingBoxes.push_back(currentSlidingWindow);
		    allWinnerProbabilities.push_back(winnerProbability);
		    allWinnerLabels.push_back(winnerObjectLabel);
		    if (DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer) //Get the probability for "all objects / unknown object" category if the Dempster-Shafer decision fusion is enabled
		      allUnknownProbabilities.push_back(getAllObjectsProbability());
		  }
	      }
	}

      //Obtain the image in the next (up and smaller) level of image pyramid (by first blurring via Gaussian filtering and then downsizing the image by half in each dimension
      pyrDown(pyramidImage, pyramidImage);
      if(REJECT_ABNORMAL_DISTANCES && useDepth)
	pyrDown(depthPyramid, depthPyramid);
      if(REJECT_INTENSITY_BASED)
	pyrDown(intensityPyramid, intensityPyramid);
    }

  //Performing non-maximum suppression on the detected bounding boxes
  convertedWinnerProbabilities = vector<float>(allWinnerProbabilities.begin(), allWinnerProbabilities.end());
  remainingBBoxIndices = NonMaximumSuppression(allBoundingBoxes, convertedWinnerProbabilities, DISCARD_CLOSE_IDENTICAL_BOUNDING_BOXES, allWinnerLabels);

  //Constructing the vector of the detected objects
  for(BBoxCounter = 0; BBoxCounter < remainingBBoxIndices.size(); BBoxCounter++)
    {
      detectedObjectsInfo.push_back(objectInfo());

      detectedObjectsInfo[BBoxCounter].classProbabilities = allProbabilities[remainingBBoxIndices[BBoxCounter]];
      detectedObjectsInfo[BBoxCounter].boundingBox = static_cast<Rect2d>(allBoundingBoxes[remainingBBoxIndices[BBoxCounter]]);
      detectedObjectsInfo[BBoxCounter].width = detectedObjectsInfo[BBoxCounter].boundingBox.width;
      detectedObjectsInfo[BBoxCounter].height = detectedObjectsInfo[BBoxCounter].boundingBox.height;
      detectedObjectsInfo[BBoxCounter].centroid = Point(detectedObjectsInfo[BBoxCounter].boundingBox.x + detectedObjectsInfo[BBoxCounter].width / 2, detectedObjectsInfo[BBoxCounter].boundingBox.y + detectedObjectsInfo[BBoxCounter].height / 2);
      detectedObjectsInfo[BBoxCounter].label = allWinnerLabels[remainingBBoxIndices[BBoxCounter]];
      detectedObjectsInfo[BBoxCounter].winnerProbability = allWinnerProbabilities[remainingBBoxIndices[BBoxCounter]];

      //Get the probability for "all objects / unknown object" category if the Dempster-Shafer decision fusion is enabled
      if (DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer)
	detectedObjectsInfo[BBoxCounter].unknownProbability = allUnknownProbabilities[remainingBBoxIndices[BBoxCounter]];

      //Check if the classifier for the main view is sufficiently confident about the winner class (by comparing the maximum probability to the second max)
      detectedObjectsInfo[BBoxCounter].isConfirmed = validateRecognition.checkConfidence(detectedObjectsInfo[BBoxCounter].classProbabilities, detectedObjectsInfo[BBoxCounter].unknownProbability);
    }

  return detectedObjectsInfo;
}

//*****************************************
//Function to detect objects based on the Mixture of Gaussians background subtraction technique
vector<objectInfo> & Detection::BackgroundSubtraction(Mat & input_image, int & cameraNumber, Mat * depthImage)
{
  //Generating the grayscale intensity channel from the BGR image
  cvtColor(input_image, grayImage, CV_BGR2GRAY);

  //Apply the Mixture of Gaussians background subtraction
  foregroundImage = MixtureOfGaussians(grayImage);

  //Filter out of range depths, if enabled
  if(REJECT_ABNORMAL_DISTANCES && useDepth)
    {
      depthMask1 = (* depthImage) > MIN_DEPTH;
      depthMask2 = (* depthImage) < MAX_DEPTH;
      cv::bitwise_and(depthMask1, depthMask2, depthMask);
      cv::bitwise_and(foregroundImage, depthMask, foregroundImage);
    }

  //Reject areas based on their intensities, if enabled
  if(REJECT_INTENSITY_BASED)
    {
      intensityMask = grayImage > intensityThreshold;
      cv::bitwise_and(foregroundImage, intensityMask, foregroundImage);
    }


  //Process the foreground image by cleaning small noise
  foregroundImage = ProcessForeground(foregroundImage, cameraNumber);

  //Call the function to detect and classify objects
  detectedObjectsInfo = Detect_fromForegroundMap(input_image, foregroundImage, cameraNumber);

  return detectedObjectsInfo;
}

//*****************************************
//Function to detect objects based on intensity thresholding
vector<objectInfo> & Detection::IntensityThresholding(Mat & input_image, int & cameraNumber, Mat * depthImage)
{
  //Generating the grayscale intensity channel from the BGR image
  cvtColor(input_image, grayImage, CV_BGR2GRAY);

  //Apply the intensity-based segmentation by comparing the difference between the intensity value with intensity 0 (black)
  cv::absdiff(grayImage, 0, foregroundImage);
  foregroundImage = foregroundImage > intensityThreshold;

  //Filter out of range depths, if enabled
  if(REJECT_ABNORMAL_DISTANCES && useDepth)
    {
      depthMask1 = (* depthImage) > MIN_DEPTH;
      depthMask2 = (* depthImage) < MAX_DEPTH;
      cv::bitwise_and(depthMask1, depthMask2, depthMask);
      cv::bitwise_and(foregroundImage, depthMask, foregroundImage);
    }

  //Process the foreground image by cleaning small noise
  foregroundImage = ProcessForeground(foregroundImage, cameraNumber);

  //Call the function to detect and classify objects
  detectedObjectsInfo = Detect_fromForegroundMap(input_image, foregroundImage, cameraNumber);

  return detectedObjectsInfo;
}

//*****************************************
//Function to detect blobs in a foreground map based on the Connected Components technique
vector<objectInfo> & Detection::ConnectedComponents_BlobDetection(Mat & input_BGRImage, Mat & input_foregroundImage, int & cameraNumber)
{
  //Clear the vector of detected blobs
  detectedBlobsInfo.clear();

  //Discard the boundary regions of the input foreground image, if enabled
  if(REJECT_INPUT_IMAGE_BOUNDARIES)
    {
      boundaryMask = Mat::zeros(input_foregroundImage.size(), input_foregroundImage.type());
      undiscardedRegion = Rect((int) (input_foregroundImage.size().width * (discardRatio / 2.0f)), (int) (input_foregroundImage.size().height * (discardRatio / 2.0f)), (int) (input_foregroundImage.size().width * (1 - discardRatio)), (int) (input_foregroundImage.size().height * (1 - discardRatio)));
      rectangle(boundaryMask, undiscardedRegion, 255, CV_FILLED);
      input_foregroundImage.copyTo(inputForeground, boundaryMask);
    }
  else
    inputForeground = input_foregroundImage;

  //Connected Components
  numberOfObjects = connectedComponentsWithStats(inputForeground, labels, stats, centroids);

  //Fill the vector of object information with the obtained data from the blob detector
  for(countComponents = 1; countComponents < numberOfObjects; countComponents++)
    {
      //If the object size is out of the normal range and that kind of rejection is enabled skip the current iteration
      objectPixelNumber = stats.at<int>(countComponents, CC_STAT_AREA);
      if ((REJECT_ABNORMAL_OBJECT_SIZES == true) && ((objectPixelNumber < MIN_OBJECT_SIZE) || (objectPixelNumber > MAX_OBJECT_SIZE)))
	continue;

      currentBlobInfo.width = stats.at<int>(countComponents, CC_STAT_WIDTH);
      currentBlobInfo.height = stats.at<int>(countComponents, CC_STAT_HEIGHT);
      currentBlobInfo.centroid = (Point) centroids.at<Point2d>(countComponents);

      //Determine bounding box of the object
      ROI_Width = stats.at<int>(countComponents, CC_STAT_WIDTH);
      ROI_Height = stats.at<int>(countComponents, CC_STAT_HEIGHT);
      ROI_Left = stats.at<int>(countComponents, CC_STAT_LEFT); // + 0.2 * ROI_Width;
      ROI_Top = stats.at<int>(countComponents, CC_STAT_TOP); // + 0.2 * ROI_Height;
      currentBlobInfo.boundingBox = cv::Rect2d(ROI_Left, ROI_Top, ROI_Width, ROI_Height);

      detectedBlobsInfo.push_back(currentBlobInfo);
    }

  //Displaying the detected objects and the color map of the detected blobs if mid-process displaying is enabled
  if(DISPLAY_MID_PROCESS_FRAMES == true)
    {
      //Obtain the image containing the detected objectes only (backgroud is blacked out)
      mask = (labels > 0);
      input_BGRImage.copyTo(allObjectsRGB, mask);

      //Display the detected objects in a black background
      windowName.str(string());
      windowName << "Object Detection - Camera #" << (cameraNumber + 1);
      imshow(windowName.str(), allObjectsRGB);

      //Create a color map of detected blobs
      objectsColored = Mat::zeros(grayImage.rows, grayImage.cols, CV_8UC3);
      countComponents = 1;
      while(countComponents < numberOfObjects)
	{
	  mask = (labels == countComponents);
	  objectsColored.setTo(Scalar(randomColor.uniform(0,255), randomColor.uniform(0,255), randomColor.uniform(0,255)), mask);

	  countComponents++;
	}

      //Show the color map
      windowName.str(string());
      windowName << "Blob Detection - Camera #" << (cameraNumber + 1);
      imshow(windowName.str(), objectsColored);
    }

  return detectedBlobsInfo;
}

//*****************************************
//Function to implement mixture of gaussians background subtraction
Mat & Detection::MixtureOfGaussians(Mat & inputImage)
{
  MoGBackSub->apply(inputImage, Mog_foregroundImage, Mog_learningRate);

  return Mog_foregroundImage;
}

//*****************************************
//Function to process foreground images
Mat & Detection::ProcessForeground(Mat & input_foreground, int & cameraNumber)
{
  //Displaying the foreground image if mid-process display is enabled
  if (DISPLAY_MID_PROCESS_FRAMES == true)
    {
      windowName.str(string());
      windowName << "Background Subtraction - Camera #" << (cameraNumber + 1);
      imshow(windowName.str(), input_foreground);
    }

  //Cleaning the foreground map by binary morphological operations
  morphologyEx(input_foreground, foregroundImage_cleaned, MORPH_OPEN, structuringElement);
  morphologyEx(foregroundImage_cleaned, foregroundImage_cleaned, MORPH_CLOSE, structuringElement);

  //Displaying the cleaned up foreground images if mid-process display is enabled
  if (DISPLAY_MID_PROCESS_FRAMES == true)
    {
      windowName.str(string());
      windowName << "Background Subtraction - Cleaned - Camera #" << (cameraNumber + 1);
      imshow(windowName.str(), foregroundImage_cleaned);
    }

  return foregroundImage_cleaned;
}

//*****************************************
//Function to detect and classify objects given a foreground map
vector<objectInfo> & Detection::Detect_fromForegroundMap(Mat & input_BGRImage, Mat & input_foregroundImage, int & cameraNumber)
{
  //Detect objects (blobs) in the foreground image
  detectedBlobsInfo = ConnectedComponents_BlobDetection(input_BGRImage, input_foregroundImage, cameraNumber);

  //Classify each detected object and update the vector of objects information
  for(objectCounter = 0; objectCounter < detectedBlobsInfo.size(); objectCounter++)
    {
      //Crop the image to contain only the current object
      objectImage = input_BGRImage(detectedBlobsInfo[objectCounter].boundingBox);

      //Classify object
      detectedBlobsInfo[objectCounter].classProbabilities = Classify(objectImage);

      //Find the winner label
      FindWinnerClass(detectedBlobsInfo[objectCounter].classProbabilities, detectedBlobsInfo[objectCounter].label, detectedBlobsInfo[objectCounter].winnerProbability, & winnerIndex);

      //Filter objects with insufficient winner probability or with Background class label
      if((detectedBlobsInfo[objectCounter].label == BACKGROUND_CLASS_NAME) || ((! DIFFERENT_PROBABILITY_THRESHOLD_PER_CLASS) && (detectedBlobsInfo[objectCounter].winnerProbability < GLOBAL_OBJECT_PROBABILITY_THRESHOLD)) || (DIFFERENT_PROBABILITY_THRESHOLD_PER_CLASS && (detectedBlobsInfo[objectCounter].winnerProbability < OBJECT_PROBABILITY_THRESHOLD_PER_CLASS[winnerIndex])))
	continue;
      else
	detectedObjectsInfo.push_back(detectedBlobsInfo[objectCounter]);

      //Get the probability for "all objects / unknown object" category if the Dempster-Shafer decision fusion is enabled
      if (DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer)
	detectedObjectsInfo.back().unknownProbability = getAllObjectsProbability();

      //Check if the classifier for the main view is sufficiently confident about the winner class (by comparing the maximum probability to the second max)
      detectedObjectsInfo.back().isConfirmed = validateRecognition.checkConfidence(detectedObjectsInfo.back().classProbabilities, detectedObjectsInfo.back().unknownProbability);
    }

  return detectedObjectsInfo;
}

//*****************************************
//Function to perform non-maximum suppression. It returns the vector of indices of the remaining bounding boxes
vector<int> Detection::NonMaximumSuppression(vector<Rect> boundingBoxes, vector<float> scores, bool discardcloseBBOxes, vector<string> labels)
{
  //Check if the size of the two input vectors are equal
  if(boundingBoxes.size() != scores.size())
    {
      cerr << "\nError in NonMaximumSuppression. The size of two input vectors are inequal.\n" << endl;
      exit(EXIT_FAILURE);
    }

  //Perform non-maximum suppression
  cv:dnn::NMSBoxes(boundingBoxes, scores, 0, NON_MAXIMUM_SUPPRESSION_OVERLAP_THRESHOLD, remainingBBoxIndices, 1, NON_MAXIMUM_SUPPRESSION_OBJECT_NUMBER);

  //Discard identical bounding boxes in close proximity of each other, if enabled
  if(discardcloseBBOxes)
    {
      //Check if the size of the vector of labels is equal to the two other input vectors
      if(labels.size() != scores.size())
	{
	  cerr << "\nError in NonMaximumSuppression. The size of two input vectors are inequal.\n" << endl;
	  exit(EXIT_FAILURE);
	}

      //Set the initial usage of all remaining bounding boxes to true
      remainingBBoxUsage = vector<bool>(remainingBBoxIndices.size(), true);

      //Compare all remaining bounding boxes to see for each comparison pair if their labels are identical and are close enough to remove one of them
      for(bbox_counter1 = 0; bbox_counter1 < static_cast<int>(remainingBBoxIndices.size() - 1); bbox_counter1++)
	{
	  //Skip the outer loop if the current bounding box is discarded
	  if(remainingBBoxUsage[bbox_counter1] == false)
	    continue;

	  for(bbox_counter2 = bbox_counter1 + 1; bbox_counter2 < remainingBBoxIndices.size(); bbox_counter2++)
	    {
	      //Skip the inner loop if the current bounding box is discarded
	      if(remainingBBoxUsage[bbox_counter2] == false)
		continue;

	      //Check if the two labels are identical
	      if(labels[remainingBBoxIndices[bbox_counter1]] == labels[remainingBBoxIndices[bbox_counter2]])
		{
		  //Check the minimum distance between the two bounding boxes
		  x_left[0] = boundingBoxes[remainingBBoxIndices[bbox_counter1]].x;
		  x_right[0] = boundingBoxes[remainingBBoxIndices[bbox_counter1]].x + boundingBoxes[remainingBBoxIndices[bbox_counter1]].width;
		  y_top[0] = boundingBoxes[remainingBBoxIndices[bbox_counter1]].y;
		  y_bottom[0] = boundingBoxes[remainingBBoxIndices[bbox_counter1]].y + boundingBoxes[remainingBBoxIndices[bbox_counter1]].height;

		  x_left[1] = boundingBoxes[remainingBBoxIndices[bbox_counter2]].x;
		  x_right[1] = boundingBoxes[remainingBBoxIndices[bbox_counter2]].x + boundingBoxes[remainingBBoxIndices[bbox_counter2]].width;
		  y_top[1] = boundingBoxes[remainingBBoxIndices[bbox_counter2]].y;
		  y_bottom[1] = boundingBoxes[remainingBBoxIndices[bbox_counter2]].y + boundingBoxes[remainingBBoxIndices[bbox_counter2]].height;

		  distanceArray[0] = abs(x_left[0] - x_left[1]);
		  distanceArray[1] = abs(x_left[0] - x_right[1]);
		  distanceArray[2] = abs(x_right[0] - x_left[1]);
		  distanceArray[3] = abs(x_right[0] - x_right[1]);
		  distanceArray[4] = abs(y_top[0] - y_top[1]);
		  distanceArray[5] = abs(y_top[0] - y_bottom[1]);
		  distanceArray[6] = abs(y_bottom[0] - y_top[1]);
		  distanceArray[7] = abs(y_bottom[0] - y_bottom[1]);

		  minDistance = * std::min_element(distanceArray, distanceArray + 8);

		  //Check if the distance is more than a threshold. If so, discard one of the two bounding boxes
		  if(minDistance < BOUNDING_BOX_PROXIMITY_DISTANCE)
		    if(scores[remainingBBoxIndices[bbox_counter1]] > scores[remainingBBoxIndices[bbox_counter2]])
		      {
			remainingBBoxUsage[bbox_counter2] = false;
		      }
		    else
		      {
			remainingBBoxUsage[bbox_counter1] = false;
			break;
		      }
		}
	    }
	}

      //Refine the vector of the remaining bounding boxes
      proximityRefinedBoundingBoxes.clear();
      for(bbox_counter1 = 0; bbox_counter1 < remainingBBoxIndices.size(); bbox_counter1++)
	if(remainingBBoxUsage[bbox_counter1] == true)
	  proximityRefinedBoundingBoxes.push_back(remainingBBoxIndices[bbox_counter1]);
      remainingBBoxIndices = proximityRefinedBoundingBoxes;
    }

  return remainingBBoxIndices;
}


