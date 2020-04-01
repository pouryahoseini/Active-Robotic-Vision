/*
 * main.cpp
 *
 *  Created on: Aug 21, 2016
 *      Author: pourya
 */

//Headers
#include <iostream>
#include <cstdio>
#include <string>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <csignal>
#include <unistd.h>
#include <sys/types.h>
#include <atomic>

#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>

#include <Python.h>
#include <boost/numpy.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>

#include "GlobalDefinitions.h"
#include "Timer.h"
#include "Display.h"
#include "Termination.h"
#include "ROS.h"
#include "Snapshot.h"
#include "VideoRecorder.h"
#include "Localization3D.h"
#include "ConfidenceCheck.h"
#include "Matching.h"
#include "DataFusion.h"
#include "Detection.h"
#include "SimulationWorld.h"
#include "RobotMovement.h"
#include "Preprocessing.h"
#include "VisionTraining.h"
#include "PlannerTraining.h"
#include "Planning.h"

//Namespaces being used
using namespace std;
using namespace cv;


//Main function of the whole project
int main(int argc, char ** argv)
{
  /************** Preliminary operations **************/
  //Setting termination conditions
  signal(SIGINT, terminationHandler);

  //Initialize Python
  setenv("PYTHONPATH", "./SourceCode", 1);
  Py_Initialize();

  //Initialize python::boost::numpy
  boost::numpy::initialize();

  /************** Object detector/classifier training **************/
  //Declare a run mode variable and initialize it to the test mode
  runModes runMode = runModes::Test;

  //Check for the first user argument
  if ( (argc > 1) && (! strcmp(argv[1], "training") || ! strcmp(argv[1], "TRAINING")  || ! strcmp(argv[1], "train") || ! strcmp(argv[1], "TRAIN")) )
    {
      //Check for the existence of the second argument
      if(argc <= 2)
	{
	  fprintf(stderr, "\n\nError: The training mode is not specified.\nExiting...\n\n");
	  exit(EXIT_FAILURE);
	}

      //Decide on the training mode
      if((! strcmp(argv[2], "vision")) || (! strcmp(argv[2], "VISION"))) //Training vision
	{
	  //Change the run mode
	  runMode = runModes::Training_Vision;

	  //train the classifier
	  VisionTraining visionTrainer(CLASSIFIER_TYPE);
	  visionTrainer.Train();

	  //print message that training is finished successfully and exit
	  printf("\n****\nTraining finished.\nExiting the porogram.\n****\n\n");
	  return EXIT_SUCCESS;
	}
      else if((! strcmp(argv[2], "locomotion")) || (! strcmp(argv[2], "LOCOMOTION"))) //Training locomotion
	{
	  //Change the run mode
	  runMode = runModes::Training_Locomotion;

	  //Change the active vision mode to intelligent. This actually changes the value of ACTIVE_VISION_MODE
	  activeVisionModes & tempActiveVisionMode = const_cast<activeVisionModes &>(ACTIVE_VISION_MODE);
	  tempActiveVisionMode = activeVisionModes::Intelligent;
	}
      else //Undefined training mode
	{
	  fprintf(stderr, "\n\nError: Unknown training mode.\nExiting...\n\n");
	  exit(EXIT_FAILURE);
	}
 }

  /************** Preliminary operations - continued **************/
  //Initilizing the ROS system
  int ros_argc = 0;
  ros::init(ros_argc, NULL, ROS_NODE_NAME);

  /************** Declarations **************/
  //General declarations
  int key, numberOfCameras, numberOfObjects[3], numberOfAllDetectedObjects[2], objectCounter;
  bool secondaryCameraUsed = false;

  //Timing declarations
  Timer timer1;

  //Declare image subscribers and data publisher to ROS
  /*Deciding on the input image topics */
  const char * ROS_image1_topic, * ROS_image2_topic, * ROS_depth_topic;
  ROS_image1_topic = SIMULATION_WORLD_ENABLE ? ROS_IMAGE1_TOPIC_SIMULATION : ROS_IMAGE1_TOPIC_REAL_WORLD;
  ROS_image2_topic = SIMULATION_WORLD_ENABLE ? ROS_IMAGE2_TOPIC_SIMULATION : ROS_IMAGE2_TOPIC_REAL_WORLD;
  ROS_depth_topic = SIMULATION_WORLD_ENABLE ? ROS_DEPTH_TOPIC_SIMULATION : ROS_DEPTH_TOPIC_REAL_WORLD;
  string camera1_frameID, camera2_frameID;
  camera1_frameID = SIMULATION_WORLD_ENABLE ? CAMERA1_FRAME_ID_SIMULATION : CAMERA1_FRAME_ID_REAL_WORLD;
  camera2_frameID = SIMULATION_WORLD_ENABLE ? CAMERA2_FRAME_ID_SIMULATION : CAMERA2_FRAME_ID_REAL_WORLD;

  ROSDataPublisher ROSLabelLocationPublisher(camera1_frameID);
  ROSImageSubscriber ROSBGRSubscriber1(ROS_image1_topic, cameraTypes::colorCamera1), ROSDepthSubscriber(ROS_depth_topic, cameraTypes::depthCamera);
  ROSImageSubscriber ROSBGRSubscriber2(ROS_image2_topic, cameraTypes::colorCamera2), ROSBGRSubscriber_outsideView(ROS_OUTSIDE_VIEW_TOPIC, cameraTypes::outsideCamera);

  //Declaring images taken from input cameras
  Mat depthImage, originalDepthImage, BGRImage[2], originalBGRImage[2], outputImage[3], outsideView;

  //Declarations for image preprocessing
  Preprocessing preprocessor;

  //Declarations for the simulation
  SimulationWorld simulationWorld1;
  atomic<bool> objectPlacementInProgress(false);
  FrameComparison frameComparator[2];
  bool frameChanged[2] = {true, true};

  //Declarations for object detection
  Detection detector(DETECTOR_TYPE, CLASSIFIER_TYPE);
  vector<objectInfo> objectsInfo[3];

  //Declaring a localization class
  Localization3D cameraCoordinate1;

  //Declarations for the reliability check
  bool uncertainFound, atLeastOneDetection_beforeRespawn = false;

  //Declarations for hand motion planning
  Planning handPlanner;
  bool planningSuccessful;

  //Declarations for the robot hand movement
  atomic<bool> handMovementInProgress(false), handMovementFinished(false);
  ros::AsyncSpinner spinner(1); //Declare a ROS asynchronous spinner
  RobotMovement PR2Mover(spinner);
  float jointAngles[5] = {0};
  bool atLeastOneDetection_beforeReturn = false, secondaryViewDetection_lastRound = false, secondaryViewDetection = false;

  //Declarations for detection lingering
  atomic<time_t> lingerStartTime;
  time_t handWaitStartTime;

  //Declarations for object matching
  vector<Point> convertedCentroids;
  Point3d convertedCentroidLocation3D;
  Point convertedCentroidLocation2D;
  Matching twoCamerasMatcher;
  bool matchFound;
  vector<Vec2i> MatchList;
  vector<Rect2d> secondaryView_boundingBoxes;

  //Declarations for decision fusion
  DataFusion decisionCombiner;
  ConfidenceCheck recognitionValidator;
  int associatedBoundingBox;

  //Declarations for tracking
  TrackingManagement tracker;
  bool detectObjects = true, trackingCertain[2] = {true, true};
  int trackingCounter = 0, trackingFailureCounter = 0;
  vector<objectInfo> allDetectedObjectsInfo[2];

  //Declarations for display purposes
  Display displayOutput;
  Mat matchingImage[2], recognitionOnlyImage[2], amalgamatedImage;

  //Declarations for output storage
  Snapshot snapshot1;
  VideoRecorder videoRecorder1;

  //Declarations for publishing to ROS
  int numberOfDetectedObjects = 0;
  vector<Point3d> objectLocations;
  vector<string> objectLabels;

  /************* Set up some camera settings *************/
  //Initialize secondary camera's frame to zero
  BGRImage[SECONDARY_CAMERA] = Mat::zeros(Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3);
  originalBGRImage[SECONDARY_CAMERA] = BGRImage[SECONDARY_CAMERA].clone();
  outputImage[SECONDARY_CAMERA] = originalBGRImage[SECONDARY_CAMERA].clone();

  //identify number of cameras present
  if(ROS_IMAGE2_ENABLE == true)
    numberOfCameras = 2;
  else
    numberOfCameras = 1;

  /************* Simulation: Run the initial tasks *************/
  //Move the robot to its default gesture
  if(ALLOW_ROBOT_MOVEMENT || SIMULATION_WORLD_ENABLE)
    PR2Mover.runInitialTasks();

  //Spawn objects in their initial locations
  if(SIMULATION_WORLD_ENABLE)
    simulationWorld1.runInitialTasks();

  /************* Start up ROS *************/
  //Wait until ROS is "warmed up"
  /* Note: Not getting proper frames from actual robot cameras in a ROS system is common in practice */
  preprocessor.waitForProperFrames(& ROSBGRSubscriber1, & ROSBGRSubscriber2, & ROSDepthSubscriber);

  /************* Simulation: Object spawning for the first time *************/
  if(SIMULATION_WORLD_ENABLE)
    {
      std::thread(& SimulationWorld::spawnNextLayout, & simulationWorld1, runMode, std::ref(objectPlacementInProgress), std::ref(lingerStartTime), 0).detach();
      atLeastOneDetection_beforeRespawn = false;
    }

  /************* Creating display windows *************/
  displayOutput.createWindows();

  /************* Main loop *************/
  //Starting the main loop of the program
  do
    {
      /************* Frame capture *************/
      //Capture frames from cameras in a ROS environment
      depthImage = ROSDepthSubscriber.getImageFromROS();
      BGRImage[MAIN_CAMERA] = ROSBGRSubscriber1.getImageFromROS();
      if (ROS_IMAGE2_ENABLE == true)
	BGRImage[SECONDARY_CAMERA] = ROSBGRSubscriber2.getImageFromROS();

      if(SIMULATION_WORLD_ENABLE)
	outsideView = ROSBGRSubscriber_outsideView.getImageFromROS();

      /************* Preprocessing *************/
      preprocessor.preprocess(BGRImage, depthImage, originalBGRImage, originalDepthImage);

      //Make a copy of the main view's original image on the fused view's image
      originalBGRImage[MAIN_CAMERA].copyTo(outputImage[FUSED_VIEW]);

      //Save BGR images to show mid-process results later if it is enabled
      if (DISPLAY_MID_PROCESS_FRAMES == true)
	{
          matchingImage[MAIN_CAMERA] = originalBGRImage[MAIN_CAMERA].clone();
          recognitionOnlyImage[MAIN_CAMERA] = originalBGRImage[MAIN_CAMERA].clone();

          if(ROS_IMAGE2_ENABLE == true)
            {
              matchingImage[SECONDARY_CAMERA] = originalBGRImage[SECONDARY_CAMERA].clone();
              recognitionOnlyImage[SECONDARY_CAMERA] = originalBGRImage[SECONDARY_CAMERA].clone();
            }
	}

      /************* Object detection and tracking (Main Camera) *************/
      /*In case of the simulation, check to see if the current frame is different from the previous one
       * This is to save efforts on classification, because in the simualtion there is no noise when everything is static */
      if(SIMULATION_WORLD_ENABLE)
	frameChanged[MAIN_CAMERA] |= frameComparator[MAIN_CAMERA].compareFrames(BGRImage[MAIN_CAMERA]);

      if(frameChanged[MAIN_CAMERA])  //In case of the real world tests, frameChanged is always true
	{
	  if(detectObjects) //Detection
	    {
	      //Detect objects in the main camera view
	      objectsInfo[MAIN_CAMERA] = detector.Detect(BGRImage[MAIN_CAMERA], MAIN_CAMERA, & depthImage);

	      //At this stage the all the detected objects are considered trackable, so the finalized objectsInfo vector is equal to the library of all the detected objects allDetectedObjectsInfo
	      allDetectedObjectsInfo[MAIN_CAMERA] = objectsInfo[MAIN_CAMERA];
	      numberOfAllDetectedObjects[MAIN_CAMERA] = allDetectedObjectsInfo[MAIN_CAMERA].size();

	      //Reset the flag to memorize frame changes since the last detection
	      if(SIMULATION_WORLD_ENABLE)
		frameChanged[MAIN_CAMERA] = false;
	    }
	  else //Tracking
	    {
	      //Track objects in the main view
	      trackingCertain[MAIN_CAMERA] = tracker.updateTracker(BGRImage[MAIN_CAMERA], objectsInfo[MAIN_CAMERA], allDetectedObjectsInfo[MAIN_CAMERA]);
	    }

	  //Save the number of objects in the main view
	  numberOfObjects[MAIN_CAMERA] = objectsInfo[MAIN_CAMERA].size();
	}

      /************* 3D localization *************/
      //If 3D localization is enabled, find the 3D position of objects with respect to the main camera
      if (ENABLE_3D_WORLD_LOCALIZATION == true)
	for(objectCounter = 0; objectCounter < numberOfObjects[MAIN_CAMERA]; objectCounter++)
	  //Get the 3D location of the centroid of object in the 3D space
	  cameraCoordinate1.get3DWorldPoint(objectsInfo[MAIN_CAMERA][objectCounter].centroid, objectsInfo[MAIN_CAMERA][objectCounter].position3D);

      //Copy the main view's detections to the fused view
      objectsInfo[FUSED_VIEW] = objectsInfo[MAIN_CAMERA];
      numberOfObjects[FUSED_VIEW] = numberOfObjects[MAIN_CAMERA];

      /************* Reliability check and object placement *************/
      //Bypass hand movement, secondary camera classifications, object matching, and decision fusion if 3D localization or secondary camera are not enabled
      if((! ROS_IMAGE2_ENABLE) || (! ENABLE_3D_WORLD_LOCALIZATION))
	{
	  if(SIMULATION_WORLD_ENABLE)
	    {
	      //This flag enforces at least one detection (not tracking) before another set of objects are spawned
	      atLeastOneDetection_beforeRespawn = (atLeastOneDetection_beforeRespawn || detectObjects) && (! objectPlacementInProgress);

	      //Spawn objects on the table in simulation mode if the time is up
	      if((difftime(time(NULL), lingerStartTime) > DETECTION_LINGER_TIME) && (! objectPlacementInProgress) && atLeastOneDetection_beforeRespawn)
		{
		  std::thread(& SimulationWorld::spawnNextLayout, & simulationWorld1, runMode, std::ref(objectPlacementInProgress), std::ref(lingerStartTime), 0).detach();
		  atLeastOneDetection_beforeRespawn = false;
		}
	    }

	  goto BYPASS_POINT; //Shame on me! for using goto, but it was really a simplifying tool here.
	}

      //Search to see if there is any object with uncertain classification
      uncertainFound = false;
      for(objectCounter = 0; objectCounter < numberOfObjects[MAIN_CAMERA]; objectCounter++)
	if((uncertainFound |= (! objectsInfo[MAIN_CAMERA][objectCounter].isConfirmed)))
	  break;

      //Bypass hand movement, secondary camera classifications, object matching, and decision fusion if all the detections are certain and the hand should not return to its original pose
      if((! uncertainFound) && (! handMovementFinished))
	{
	  if(SIMULATION_WORLD_ENABLE)
	    {
	      //This flag enforces at least one detection (not tracking) before another set of objects are spawned
	      atLeastOneDetection_beforeRespawn = (atLeastOneDetection_beforeRespawn || detectObjects) && (! objectPlacementInProgress);

	      //Spawn objects on the table in simulation mode if the time is up
	      if((difftime(time(NULL), lingerStartTime) > DETECTION_LINGER_TIME) && (! objectPlacementInProgress) && atLeastOneDetection_beforeRespawn)
		{
		  std::thread(& SimulationWorld::spawnNextLayout, & simulationWorld1, runMode, std::ref(objectPlacementInProgress), std::ref(lingerStartTime), 0).detach();
		  atLeastOneDetection_beforeRespawn = false;
		}
	    }

	  goto BYPASS_POINT;
	}

      /************* Move hand *************/
      if(ALLOW_ROBOT_MOVEMENT && (ACTIVE_VISION_MODE != activeVisionModes::None))
	{
	  if(! handMovementFinished) //If the hand is not completely moved to the desired pose
	    {
	      if((! handMovementInProgress) && (! objectPlacementInProgress) && detectObjects) //If the hand movement is not yet started, start it
		{
		  //Plan the hand pose
		  planningSuccessful = handPlanner.plan(jointAngles[0], jointAngles[1], jointAngles[2], jointAngles[3], jointAngles[4], objectsInfo[MAIN_CAMERA]);

		  //Check if planning was successful. If not, skip the remainder of the loop.
		  if(! planningSuccessful)
		    {cout<<endl<<"Planning unsuccessful"<<endl;
		      //Resapwn objects in simulation after a while
		      if(SIMULATION_WORLD_ENABLE && (difftime(time(NULL), lingerStartTime) > DETECTION_LINGER_TIME))
			std::thread(& SimulationWorld::spawnNextLayout, & simulationWorld1, runMode, std::ref(objectPlacementInProgress), std::ref(lingerStartTime), 0).detach();

		      goto BYPASS_POINT;
		    } else cout<<endl<<"Planning successful"<<endl;

		  //Set hand movement in progress
		  moveInProgress_lock.lock();
		  handMovementInProgress = true;
		  moveInProgress_lock.unlock();

		  //Start the hand movement thread
		  std::thread(& RobotMovement::moveLeftHand, & PR2Mover, jointAngles[0], jointAngles[1], jointAngles[2], jointAngles[3], jointAngles[4], std::ref(handMovementFinished), true).detach();
		  atLeastOneDetection_beforeReturn = false;
		}

	      //Bypass secondary camera classifications, object matching, and decision fusion if the secondary camera mounted on the robot's hand is not yet moved to the desired pose
	      goto BYPASS_POINT;
	    }
	  else //Check the termination of the hand movement operation
	    {
	      //Save the time of the robot's hand in the desired location
	      if(handMovementInProgress)
		{
		  //Set hande movement is not in progress
		  moveInProgress_lock.lock();
		  handMovementInProgress = false;
		  moveInProgress_lock.unlock();
		}

	      //Save the starting time of a timer after a detection round to move back the hand after a while
	      if(! atLeastOneDetection_beforeReturn)
		handWaitStartTime = time(NULL);

	      //This flag enforces at least one detection (not tracking) before the hand is returned to its original place
	      atLeastOneDetection_beforeReturn = atLeastOneDetection_beforeReturn || secondaryViewDetection_lastRound;

	      //Check if it is time to return the hand to its original pose
	      if((difftime(time(NULL), handWaitStartTime) > HAND_LINGER_TIME_AFTER_DETECTION) && (! objectPlacementInProgress) && atLeastOneDetection_beforeReturn)
		{
		  //Set hand movement in progress
		  moveInProgress_lock.lock();
		  handMovementInProgress = true;
		  moveInProgress_lock.unlock();
		  moveFinished_lock.lock();
		  handMovementFinished = false;
		  moveFinished_lock.unlock();

		  //Place objects on the table if the simulation world is enabled
		  if(SIMULATION_WORLD_ENABLE)
		    {
		      std::thread(& SimulationWorld::spawnNextLayout, & simulationWorld1, runMode, std::ref(objectPlacementInProgress), std::ref(lingerStartTime), RESPAWNING_LINGER_AFTER_HAND_MOVEMENT).detach();
		      atLeastOneDetection_beforeRespawn = false;
		    }
		  atLeastOneDetection_beforeReturn = false;

		  //Move back the hand to its original pose
		  std::thread(& RobotMovement::moveHandToDefaultPose, & PR2Mover, std::ref(handMovementInProgress), false, "left_arm", "leftHand_tableTop").detach();

		  //Clear the vector of detected objects in the secondary view to prevent displaying false detections while there is no object detection nor object tracking is being done for the secondary view
		  objectsInfo[SECONDARY_CAMERA].clear();
		  numberOfObjects[SECONDARY_CAMERA] = 0;
		  allDetectedObjectsInfo[SECONDARY_CAMERA].clear();
		  secondaryCameraUsed = false;

		  //Bypass secondary camera classifications, object matching, and decision fusion if the secondary camera mounted on the robot's hand started to return to its original pose
		  goto BYPASS_POINT;
		}
	    }
	}

      /************* Object detection and tracking (Secondary Camera) *************/
      //Set the indicator flag for the secondary camera usage to true
      secondaryCameraUsed = true;

      /*In case of the simulation, check to see if the current frame is different from the previous one
       * This is to save efforts on classification, because in the simualtion there is no noise when everything is static */
      if(SIMULATION_WORLD_ENABLE)
	frameChanged[SECONDARY_CAMERA] |= frameComparator[SECONDARY_CAMERA].compareFrames(BGRImage[SECONDARY_CAMERA]);

      if(frameChanged[SECONDARY_CAMERA]) //In case of the real world tests, frameChanged is always true
	{
	  if(detectObjects) //Detection
	    {
	      //Detect objects in the secondary camera view
	      objectsInfo[SECONDARY_CAMERA] = detector.Detect(BGRImage[SECONDARY_CAMERA], SECONDARY_CAMERA);

	      //At this stage, all the detected objects are considered trackable, so the finalized objectsInfo vector is equal to the library of all the detected objects allDetectedObjectsInfo
	      allDetectedObjectsInfo[SECONDARY_CAMERA] = objectsInfo[SECONDARY_CAMERA];
	      numberOfAllDetectedObjects[SECONDARY_CAMERA] = allDetectedObjectsInfo[SECONDARY_CAMERA].size();

	      //Reset the flag to memorize frame changes since the last detection
	      if(SIMULATION_WORLD_ENABLE)
		frameChanged[SECONDARY_CAMERA] = false;

	      //Indicate that the secondary view detection was done in this iteration
	      secondaryViewDetection = true;
	    }
	  else //Tracking
	    {
	      //Track objects in the secondary view
	      trackingCertain[SECONDARY_CAMERA] = tracker.updateTracker(BGRImage[SECONDARY_CAMERA], objectsInfo[SECONDARY_CAMERA], allDetectedObjectsInfo[SECONDARY_CAMERA]);
	    }

	  //Save the number of objects in the secondary view
	  numberOfObjects[SECONDARY_CAMERA] = objectsInfo[SECONDARY_CAMERA].size();
	}

      /************* Object matching *************/
      //Clear the list of converted centroids (in pixel coordinate of the secondary view)
      convertedCentroids.clear();

      //Transform the 3D position of centroid of the objects in the main view to the pixel coordinate of the secondary view
      for(objectCounter = 0; objectCounter < numberOfObjects[MAIN_CAMERA]; objectCounter++)
	{
	  //Transform the 3D location from the main view to the secondary camera view
	  cameraCoordinate1.transformCameraCoordinate(objectsInfo[MAIN_CAMERA][objectCounter].position3D, convertedCentroidLocation3D, camera1_frameID, camera2_frameID);

	  //Convert the transformed 3D location to pixel coordinate of the secondary camera
	  cameraCoordinate1.get2DPixelLocation(convertedCentroidLocation3D, convertedCentroidLocation2D);

	  //Store the converted centroid locations from the main camera to secondary camera view into a vector
	  convertedCentroids.push_back(convertedCentroidLocation2D);

	  //Store the transformed centroid in the object info
	  objectsInfo[MAIN_CAMERA][objectCounter].transformedCentroid = convertedCentroidLocation2D;
	}

      //Gather all the bounding boxes in the secondary view
      secondaryView_boundingBoxes.clear();
      for(objectCounter = 0; objectCounter < numberOfObjects[SECONDARY_CAMERA]; objectCounter++)
	secondaryView_boundingBoxes.push_back(objectsInfo[SECONDARY_CAMERA][objectCounter].boundingBox);

      //Object matching
      matchFound = twoCamerasMatcher.Match(secondaryView_boundingBoxes, convertedCentroids, MatchList);

      //If no match found bypass the decision fusion stage
      if(! matchFound)
	goto BYPASS_POINT;

      //Update the objects vectors to save matching between the two views
      twoCamerasMatcher.saveMatches(MatchList, objectsInfo[MAIN_CAMERA], objectsInfo[SECONDARY_CAMERA]);

      /************* Decision fusion *************/
      for(objectCounter = 0; objectCounter < numberOfObjects[MAIN_CAMERA]; objectCounter++) //For all the detected objects in the main view
	{
	  //If the object's detection is uncertain
	  if(objectsInfo[MAIN_CAMERA][objectCounter].isConfirmed == false)
	    {
	      //Skip the loop if there is no match for the current centroid in the match list
	      if(objectsInfo[MAIN_CAMERA][objectCounter].matchFound == false)
		continue;

	      //Get the aasociated object in the secondary view to the current centroid
	      associatedBoundingBox = objectsInfo[MAIN_CAMERA][objectCounter].matchedIndex;

	      //Fuse probaility vectors
	      objectsInfo[FUSED_VIEW][objectCounter].classProbabilities = decisionCombiner.Fuse(objectsInfo[MAIN_CAMERA][objectCounter].classProbabilities, objectsInfo[SECONDARY_CAMERA][associatedBoundingBox].classProbabilities, objectsInfo[MAIN_CAMERA][objectCounter].unknownProbability, objectsInfo[SECONDARY_CAMERA][associatedBoundingBox].unknownProbability, objectsInfo[FUSED_VIEW][objectCounter].unknownProbability);

	      //Find the class with largest probability as the winner (fused view)
	      detector.FindWinnerClass(objectsInfo[FUSED_VIEW][objectCounter].classProbabilities, objectsInfo[FUSED_VIEW][objectCounter].label, objectsInfo[FUSED_VIEW][objectCounter].winnerProbability);

	      //Check if decision fusion results in enough confidence about the winner class (by comparing the maximum probability to the second max)
	      objectsInfo[FUSED_VIEW][objectCounter].isConfirmed = recognitionValidator.checkConfidence(objectsInfo[FUSED_VIEW][objectCounter].classProbabilities, objectsInfo[FUSED_VIEW][objectCounter].unknownProbability);
	    }
	}

      /************* Bypass point of operations related to the use of the secondary camera *************/
      BYPASS_POINT:

      /************* Displaying results *************/
      //Print mid-process info
      displayOutput.printOutputInfo(objectsInfo);

      //Construct the output images
      outputImage[MAIN_CAMERA] = originalBGRImage[MAIN_CAMERA].clone();
      if(ROS_IMAGE2_ENABLE)
	outputImage[SECONDARY_CAMERA] = originalBGRImage[SECONDARY_CAMERA].clone();
      displayOutput.constructOutputImages(objectsInfo, matchingImage, recognitionOnlyImage, outputImage);

      //Calculating frame rate
      timer1.getFrameRate();

      //Show amalgamated/separate frames along with the mid-process frames (if enabled)
      amalgamatedImage = displayOutput.displayAtOnce(outputImage, recognitionOnlyImage, matchingImage, & timer1, outsideView);

      /************* Storing results *************/
      //Recording videos per user request
      videoRecorder1.recordVideo(& key, originalBGRImage[MAIN_CAMERA], outputImage[MAIN_CAMERA], originalDepthImage, originalBGRImage[SECONDARY_CAMERA], outputImage[SECONDARY_CAMERA], outputImage[FUSED_VIEW], amalgamatedImage);

      //Saving a snapshot per user request and store labels and probability vectors on disk if enabled
      snapshot1.SaveSnapshot( & key, originalBGRImage[MAIN_CAMERA], depthImage, outputImage[MAIN_CAMERA], originalBGRImage[SECONDARY_CAMERA], outputImage[SECONDARY_CAMERA], outputImage[FUSED_VIEW], amalgamatedImage, objectsInfo);
      //snapshot1.SaveCalibrationSnapshot(& key, originalBGRImage[MAIN_CAMERA], originalBGRImage[SECONDARY_CAMERA]);

      /************* Publish to ROS *************/
      //If enabled, publish centroids as well as labels of the detected objects to ROS
      if(PUBLISH_TO_ROS==true)
	{
	  //Clear the vector of object labels
	  objectLabels.clear();
	  numberOfDetectedObjects = 0;

	  //Save centroid location of objects in the 3D space in vectors, if 3D localization is enabled.
	  if (ENABLE_3D_WORLD_LOCALIZATION == true)
	    {
	      objectLocations.clear();

	      //Save centroid location of object in the 3D space in a vector
	      for(objectCounter = 0; objectCounter < numberOfObjects[FUSED_VIEW]; objectCounter++) //For all the objects in the fused view
		if(((! DISCARD_UNCERTAIN_FUSED_DETECTIONS) || (objectsInfo[FUSED_VIEW][objectCounter].isConfirmed)) && (objectsInfo[FUSED_VIEW][objectCounter].label != BACKGROUND_CLASS_NAME))
		  objectLocations.push_back(objectsInfo[FUSED_VIEW][objectCounter].position3D);
	    }
	  else
	    objectLocations = vector<Point3d>(objectsInfo[FUSED_VIEW].size(), Point3d(0, 0, 0)); //Construct a maximum size vector of object locations with all values set to zero if the 3D localization is not enabled

	  //Save final (fused) object label in a vector
	  for(objectCounter = 0; objectCounter < numberOfObjects[FUSED_VIEW]; objectCounter++) //For all the objects in the fused view
	    if(((! DISCARD_UNCERTAIN_FUSED_DETECTIONS) || (objectsInfo[FUSED_VIEW][objectCounter].isConfirmed)) && (objectsInfo[FUSED_VIEW][objectCounter].label != BACKGROUND_CLASS_NAME))
	      {
		objectLabels.push_back(objectsInfo[FUSED_VIEW][objectCounter].label);

		//Increment the number of the remaining detected objects in the final (fused) view
		numberOfDetectedObjects++;
	      }

	  //Update the ROS message
	  ROSLabelLocationPublisher.dataToROS(numberOfDetectedObjects, objectLabels, objectLocations);
	}

      /************* Tracking scheduling *************/
      //Check for the tracker failure in at least one object and prepare detection in the next iteration if a specified number of consecutive failures happen
      if(((numberOfObjects[MAIN_CAMERA] != numberOfAllDetectedObjects[MAIN_CAMERA]) || (! trackingCertain[MAIN_CAMERA])) || (secondaryCameraUsed && ((numberOfObjects[SECONDARY_CAMERA] != numberOfAllDetectedObjects[SECONDARY_CAMERA]) || (! trackingCertain[SECONDARY_CAMERA]))))
	{
	  trackingFailureCounter++;
	  if(trackingFailureCounter >= TRACKER_RETRY_AFTER_FAIL)
	    {
	      trackingFailureCounter = 0;
	      trackingCounter = TRACKING_INTERVAL - 1; //Reset the tracking counter as well
	      detectObjects = true;
	      trackingCertain[MAIN_CAMERA] = (trackingCertain[SECONDARY_CAMERA] = true);
	    }
	}
      else
	trackingFailureCounter = 0;

      //Check if it is detection turn and set the flag for detection in the next iteration
      ++trackingCounter;
      trackingCounter %= (TRACKING_INTERVAL + 1);
      if(trackingCounter == TRACKING_INTERVAL)
	detectObjects = true;
      else
	detectObjects = false;

      /************* Main loop additional tasks *************/
      //Save if the secondary view detection is done in this iteration
      secondaryViewDetection_lastRound = secondaryViewDetection;
      secondaryViewDetection = false;

      //Set the indicator flag for the secondary camera usage to false
      secondaryCameraUsed = false;

      //Spin ROS once
      ros::spinOnce();

      //Wait for a keypress
      key = waitKey(WAIT_KEY_MILLISECOND);

    } while(!(key > 0 && ((key & 0xFF) == 27))); //Continue the main loop of the program if user has not hit Escape key

  //Close all the output windows
  destroyAllWindows();

  //Move the robot parts to their final gestures and strop the ROS asynchronous spinner. For safety reasons it is performed whenever the program is running in simulation mode
  if(SIMULATION_WORLD_ENABLE == true)
    {
      //PR2Mover.releaseTimeActions();
      spinner.stop();
    }

  //Shut down ROS
  ros::shutdown();

  //Finish the program normally
  return 0;
}

