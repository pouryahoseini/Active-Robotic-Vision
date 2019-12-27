/*
 * Tracking.cpp
 *
 *  Created on: Aug 23, 2018
 *      Author: pourya
 */

//Header
#include "TrackingManagement.h"

/*********************************/
//Constructor
TrackingManagement::TrackingManagement()
{
  //Decide on which tracker to use
  if(TRACKER_TYPE == TRACKER_TYPES::MIL)
    trackerCreate_ptrFunction = & TrackingManagement::Tracker_MIL_Creator;
  else if(TRACKER_TYPE == TRACKER_TYPES::MOSSE)
    trackerCreate_ptrFunction = & TrackingManagement::Tracker_MOSSE_Creator;
  else if(TRACKER_TYPE == TRACKER_TYPES::KCF)
    trackerCreate_ptrFunction = & TrackingManagement::Tracker_KCF_Creator;
  else if(TRACKER_TYPE == TRACKER_TYPES::TLD)
    trackerCreate_ptrFunction = & TrackingManagement::Tracker_TLD_Creator;
  else if(TRACKER_TYPE == TRACKER_TYPES::MedianFlow)
    trackerCreate_ptrFunction = & TrackingManagement::Tracker_MedianFlow_Creator;
  else if(TRACKER_TYPE == TRACKER_TYPES::Boosting)
    trackerCreate_ptrFunction = & TrackingManagement::Tracker_Boosting_Creator;
  else if(TRACKER_TYPE == TRACKER_TYPES::GOTURN)
    trackerCreate_ptrFunction = & TrackingManagement::Tracker_GOTURN_Creator;
}

/*********************************/
//Function to create and initialize trackers
void TrackingManagement::createAndInitilizeTrackers(Mat & input_image, vector<objectInfo> & objectsInfo)
{
  //For all the objects
  for(objectCounter = 0; objectCounter < objectsInfo.size(); objectCounter++)
    {
      //Create a tracker for each object
      objectsInfo[objectCounter].tracker_ptr = (this->* trackerCreate_ptrFunction)();

      //Initialize each object's tracker
      objectsInfo[objectCounter].isPresent = objectsInfo[objectCounter].tracker_ptr->init(input_image, objectsInfo[objectCounter].boundingBox);

      //Initially set tracking certainty true, because objects are freshly classified
      objectsInfo[objectCounter].trackingCertain = true;
    }

  return;
}

/*********************************/
//Function to update tracker and respective object details. It returns false if there is any uncertain trackings among the present objects in the tracking
bool TrackingManagement::updateTracker(Mat & input_image, vector<objectInfo> & objectsInfo, vector<objectInfo> & allDetectedObjectsInfo)
{
  //Clear the vector of tracked objects
  objectsInfo.clear();

  //For all the objects
  for(objectCounter = 0; objectCounter < allDetectedObjectsInfo.size(); objectCounter++)
    {
      //Update the tracker for each object
      allDetectedObjectsInfo[objectCounter].isPresent = allDetectedObjectsInfo[objectCounter].tracker_ptr->update(input_image, allDetectedObjectsInfo[objectCounter].boundingBox);

      //Update the related properties of the object
      allDetectedObjectsInfo[objectCounter].width = allDetectedObjectsInfo[objectCounter].boundingBox.width;
      allDetectedObjectsInfo[objectCounter].height = allDetectedObjectsInfo[objectCounter].boundingBox.height;
      allDetectedObjectsInfo[objectCounter].centroid = Point(allDetectedObjectsInfo[objectCounter].boundingBox.x + (allDetectedObjectsInfo[objectCounter].width / 2.0f), allDetectedObjectsInfo[objectCounter].boundingBox.y + (allDetectedObjectsInfo[objectCounter].height / 2.0f));

      //Add the object if it is tracked successfully
      if(allDetectedObjectsInfo[objectCounter].isPresent)
	objectsInfo.push_back(allDetectedObjectsInfo[objectCounter]);
    }

  //Enforce validity of the tracked bounding boxes that are present
  enforceBoundingboxValidity(objectsInfo);

  //Check certainty of the tracking
  if(COMPUTE_TRACKING_CERTAINTY)
    allCertain = monitorTrackerConfidence(input_image, objectsInfo);

  return allCertain;
}

/*********************************/
//Function to classify each tracking window and determine the certainty of the tracking. It returns false if there is any uncertain trackings
bool TrackingManagement::monitorTrackerConfidence(Mat & input_image, vector<objectInfo> & objectsInfo)
{
  //Set the return value to true initially
  allCertain = true;

  //Scan all the tracked bounding boxes
  for(objectCounter = 0; objectCounter < objectsInfo.size(); objectCounter++)
    {
      //Classifying the current bounding box
      currentBBoxImage = input_image(objectsInfo[objectCounter].boundingBox);
      objectProbabilities = classifier.Classify(currentBBoxImage);

      //Find the winner class and probability
      classifier.FindWinnerClass(objectProbabilities, winnerObjectLabel, winnerProbability);

      //Check the winner label against the current bounding box's label and check the probability of the winner against the threshold value for detecting an object
      if((winnerObjectLabel != objectsInfo[objectCounter].label) || (winnerProbability < GLOBAL_OBJECT_PROBABILITY_THRESHOLD))
	{
	  objectsInfo[objectCounter].trackingCertain = false;
	  allCertain = false;
	}
      else
	objectsInfo[objectCounter].trackingCertain = true;
    }

  return allCertain;
}

/*********************************/
//Function to enforce the validity of a tracked bounding box
void TrackingManagement::enforceBoundingboxValidity(vector<objectInfo> & objectsInfo)
{
  validBBoxes.clear();

  //Iterate over all the bounding boxes
  for(bboxCounter = 0; bboxCounter < objectsInfo.size(); bboxCounter++)
    {
      //Retrieve the current bounding box
      currentBBox = objectsInfo[bboxCounter].boundingBox;

      //Enforce the validity of the horizontal starting location
      if(currentBBox.x < 0)
	{
	  currentBBox.width += currentBBox.x;
	  currentBBox.x = 0;
	}
      else if(currentBBox.x >= IMAGE_WIDTH)
	currentBBox.x = IMAGE_WIDTH - 1;

      //Enforce the validity of the vertical starting location
      if(currentBBox.y < 0)
	{
	  currentBBox.height += currentBBox.y;
	  currentBBox.y = 0;
	}
      else if(currentBBox.y >= IMAGE_HEIGHT)
	currentBBox.y = IMAGE_HEIGHT - 1;

      //Enforce the validity of width and height
      if(currentBBox.x + currentBBox.width > IMAGE_WIDTH - 1)
	currentBBox.width -= currentBBox.x + currentBBox.width - (IMAGE_WIDTH - 1);

      if(currentBBox.y + currentBBox.height > IMAGE_HEIGHT - 1)
	currentBBox.height -= currentBBox.y + currentBBox.height - (IMAGE_HEIGHT - 1);

      //Store the bounding box and the corresponding object
      if((currentBBox.width > 0) && (currentBBox.height > 0))
	{
	  objectsInfo[bboxCounter].boundingBox = currentBBox;
	  validBBoxes.push_back(objectsInfo[bboxCounter]);
	}
    }

  //Copy the vector of valid bounding boxes on the function argument
  objectsInfo = validBBoxes;

  return;
}

