/*
 * Display.h
 *
 *  Created on: Aug 27, 2016
 *      Author: pourya
 */

//Guard
#ifndef DISPLAY_H_
#define DISPLAY_H_

//Headers
#include <iostream>
#include <cstdio>
#include <thread>
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"

#include "Timer.h"

//Global definitions
#include "GlobalDefinitions.h"

//Namespaces being used
using namespace std;
using namespace cv;

//Global definitions
extern const int MAIN_CAMERA, SECONDARY_CAMERA, FUSED_VIEW;
static const int THIN_LINE = 1, THICK_LINE = 2;

//External declarations
//The function to read the list of objects from a file. Returns the number of objects
extern int ObjectListReader(vector<string> & output_objectNamesList); //Originally declared in Classification.h

//The class to display the intended results in realtime
class Display
{
public:
  //Constructor
  Display();

  //Destructor
  ~Display();

  //Function to create named windows
  void createWindows();

  //Function to display output results of the program
  void constructOutputImages(vector<objectInfo> detectedObjectsInfo[], Mat output_matchingImage[], Mat output_recognitionOnlyImage[], Mat output_BGRImage[]);

  //Function to print mid-process info about the detected objects
  void printOutputInfo(vector<objectInfo> detectedObjectsInfo[]);

  //Threaded function to display amalgamated output
  Mat & displayAmalgamatedFrame(Mat & inputMainCameraFrame, Mat & inputSecondaryCameraFrame, Mat & inputFusedFrame, Timer * timer, Mat & outsideView);

  //Threaded function to display separate outputs
  void displaySeparatedFrames(Mat & inputMainCameraFrame, Mat & inputSecondaryCameraFrame, Mat & inputFusedFrame, Timer * timer);

  //Threaded function to display mid-process images
  void displayMidProcess(Mat recognitionOnlyImage[2], Mat matchingImage[2]);

  //Function to display amalgamated/separated frames and mid process frames at once
  Mat & displayAtOnce(Mat outputImage[3], Mat recognitionOnlyImage[2], Mat matchingImage[2], Timer * timer, Mat & outsideView);

private:

  //The function to decide a color per label
  Scalar colorPerLabel(string & label);

  //Function to add a bounding box to an image
  void addBoundingBox(Mat & manipulatedImage, objectInfo & object_info, Scalar inputColor, int thickness = THICK_LINE);

  //The function to resize images to be displayed if necessary
  void resizeIfNecessary(Mat & inputMainCameraFrame, Mat & inputSecondaryCameraFrame, Mat & inputFusedFrame, Mat & outsideView);

  //The function to display fixed interface on the amalgamated window
  void constructAmalgamatedImage();

  Mat amalgamatedImage, readImage, manipulationImage, inputMainCameraFrame_confirmedSize, inputSecondaryCameraFrame_confirmedSize, inputFusedFrame_confirmedSize, emptyMat, outsideView_confirmedSize;
  int numberOfObjects, counter, labelCounter, objectCounter, cameraCounter, numberOfCameras, matchedIndex, thickness;
  Scalar backgroundColor, displayColor;
  vector<string> labels;
  vector<Scalar> colors;
  RNG randomNumber = RNG(getTickCount());
  Rect convertedBoundingBox;
};

#endif /* DISPLAY_H_ */
