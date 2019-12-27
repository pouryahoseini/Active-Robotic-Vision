/*
 * Snapshot.h
 *
 *  Created on: Mar 28, 2017
 *      Author: pourya
 */

//Guard
#ifndef SRC_SNAPSHOT_H_
#define SRC_SNAPSHOT_H_

//Declarations
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>

//Global definitions
#include "GlobalDefinitions.h"

//Namespaces being used
using namespace std;
using namespace cv;

//Global definitions
extern const int MAIN_CAMERA, SECONDARY_CAMERA, FUSED_VIEW;

//The structure to save snapshots
struct Snapshot
{
  //Constructor
  Snapshot();

  //The function to save snapshots
  void SaveSnapshot(int * key, Mat & BGRImage_raw, Mat & depthImage_raw, Mat & BGRImage_processed, Mat & BGRImage2_raw, Mat & BGRImage2_processed, Mat & BGRImageFused_processed, Mat & amalgamated_processed, vector<objectInfo> * detectedObjectsInfo);

  //The function to save snapshots for calibration
  void SaveCalibrationSnapshot(int * key, Mat & BGRImage1, Mat & BGRImage2);

private:
  //Function to extract probabilities and labels of objects to prepare their writing on a file, if enabled
  void extractClassificationInfo(vector<objectInfo> * input_detectedObjectsInfo, vector<Mat> * output_allClassProbabilities, vector<string> * output_allLabels, vector<double> * output_allUnknownProbabilities = NULL);

  //The function to copy current time into a string
  void saveTimeToString();

  //The function to store probability vectors
  void storeProbabilityVectors(char * baseFilename, vector<Mat> * allClassProbabilities, vector<string> * allLabels, vector<double> * allUnknownProbabilities = NULL);

  char snapshotFileName_BGR[100], snapshotFileName_BGR2[100], snapshotFileName_Depth[100], timeString[50], snapshotFileName_Amalgamated[100], probabilityVectorsFileName[100];
  time_t now;
  struct tm *localTime;
  vector<int> imageSaveParameters;
  Rect2d roi;
  ofstream probabilityVectorsFile;
  int counter, cameraCounter, objectCounter, numberOfCameras;
  vector<Mat> allClassProbabilities[3];
  vector<string> allLabels[3];
  vector<double> allUnknownProbabilities[2];
};

#endif /* SRC_SNAPSHOT_H_ */
