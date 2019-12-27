/*
 * Tracking.h
 *
 *  Created on: Aug 23, 2018
 *      Author: pourya
 */

//Guard
#ifndef SOURCECODE_TRACKINGMANAGEMENT_H_
#define SOURCECODE_TRACKINGMANAGEMENT_H_

//Headers
#include <cstdio>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include "GlobalDefinitions.h"
#include "Classification.h"

//Using namespaces
using namespace std;
using namespace cv;

//Class to track objects
class TrackingManagement
{
public:
  //Constructor
  TrackingManagement();

  //Function to create and initialize trackers
  void createAndInitilizeTrackers(Mat & input_image, vector<objectInfo> & objectsInfo);

  //Function to update tracker and respective object details. It returns false if there is any uncertain trackings among the present objects in the tracking
  bool updateTracker(Mat & input_image, vector<objectInfo> & objectsInfo, vector<objectInfo> & allDetectedObjectsInfo);

private:
  //Function to classify each tracking window and determine the certainty of the tracking
  bool monitorTrackerConfidence(Mat & input_image, vector<objectInfo> & objectsInfo);

  //Function to enforce the validity of a tracked bounding box
  void enforceBoundingboxValidity(vector<objectInfo> & objectsInfo);

  //Tracker creator functions
  Ptr<Tracker> Tracker_MIL_Creator() {return TrackerMIL::create();}
  Ptr<Tracker> Tracker_MOSSE_Creator() {return TrackerMOSSE::create();}
  Ptr<Tracker> Tracker_KCF_Creator() {return TrackerKCF::create();}
  Ptr<Tracker> Tracker_TLD_Creator() {return TrackerTLD::create();}
  Ptr<Tracker> Tracker_MedianFlow_Creator() {return TrackerMedianFlow::create();}
  Ptr<Tracker> Tracker_Boosting_Creator() {return TrackerBoosting::create();}
  Ptr<Tracker> Tracker_GOTURN_Creator() {return TrackerGOTURN::create();}

  int objectCounter, bboxCounter;
  Ptr<Tracker> (TrackingManagement::* trackerCreate_ptrFunction)();
  Mat objectProbabilities, currentBBoxImage;
  Classification classifier;
  string winnerObjectLabel;
  double winnerProbability;
  bool allCertain = true;
  vector<objectInfo> validBBoxes;
  Rect2d currentBBox;
};

#endif /* SOURCECODE_TRACKINGMANAGEMENT_H_ */
