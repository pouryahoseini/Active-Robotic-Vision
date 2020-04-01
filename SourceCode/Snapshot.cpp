/*
 * Snapshot.cpp
 *
 *  Created on: Mar 28, 2017
 *      Author: pourya
 */

//Declaration
#include "Snapshot.h"

//Constructor
Snapshot::Snapshot()
{
  //Setting the image saving parameters
  imageSaveParameters.push_back(CV_IMWRITE_JPEG_QUALITY);
  imageSaveParameters.push_back(SANPSHOT_QUALITY);

  //identify number of cameras present
  if(ROS_IMAGE2_ENABLE == true)
    numberOfCameras = 2;
  else
    numberOfCameras = 1;
}

/***************************/
//The function to save snapshots
void Snapshot::SaveSnapshot(int * key, Mat & BGRImage_raw, Mat & depthImage_raw, Mat & BGRImage_processed, Mat & BGRImage2_raw, Mat & BGRImage2_processed, Mat & BGRImageFused_processed, Mat & amalgamated_processed, vector<objectInfo> * detectedObjectsInfo)
{
  //saving the BGR and Depth images
  if (*key == '0')
    {
      //saving current time into base file name
      saveTimeToString();

      //checking presense of secondary camera's processed frame
      if (ROS_IMAGE2_ENABLE == true)
	{
	  //defining the file names
	  strcpy(snapshotFileName_BGR2, snapshotFileName_BGR);
	  strcat(snapshotFileName_BGR2, "-BGR-2.jpg");

	  //writing image onto permanent memory
	  imwrite(snapshotFileName_BGR2, BGRImage2_raw, imageSaveParameters);

	  //display a message to notify user saving is done
	  if (AMALGAMATED_VIEW_ENABLED == false)
	    displayStatusBar(SECONDARY_OUTPUT, "Snapshot taken (Raw Secondary Camera)", 3000);
      	}

      //defining the file names
      strcpy(snapshotFileName_Depth, snapshotFileName_BGR);
      strcat(snapshotFileName_BGR, "-BGR.jpg");
      strcat(snapshotFileName_Depth, "-Depth.jpg");

      //writing images onto permanent memory
      imwrite(snapshotFileName_BGR, BGRImage_raw, imageSaveParameters);
      imwrite(snapshotFileName_Depth, depthImage_raw, imageSaveParameters);

      //display a message to notify user saving is done
      if (AMALGAMATED_VIEW_ENABLED == false)
	displayStatusBar(MAIN_OUTPUT, "Snapshot taken (Raw Main Camera's BGR and Depth)", 3000);
      else
	displayStatusBar(AMALGAMATED_OUTPUT, "Snapshot taken (Raw BGR and Depth)", 3000);
    }
  //saving a region of interest in main camera's both BGR and depth images
  else if (*key == '9')
    {
      //saving current time into base file name
      saveTimeToString();

      //defining the file names
      strcpy(snapshotFileName_Depth, snapshotFileName_BGR);
      strcat(snapshotFileName_BGR, "-BGR_ROI.jpg");
      strcat(snapshotFileName_Depth, "-Depth_ROI.jpg");

      //selecting region of interest
      roi = selectROI("Select Region of Interest", BGRImage_raw);

      //move the opened window
      moveWindow("Select Region of Interest", 20, 20);

      //writing images onto permanent memory
      imwrite(snapshotFileName_BGR, BGRImage_raw(roi), imageSaveParameters);
      imwrite(snapshotFileName_Depth, depthImage_raw(roi), imageSaveParameters);
    }
  //saving a region of interest in secondary camera's gray/BGR image
  else if (*key == '8')
    {
      //saving current time into base file name
      saveTimeToString();

      //defining the file names
      strcat(snapshotFileName_BGR, "-BGR-2_ROI.jpg");

      //selecting region of interest
      roi = selectROI("Select Region of Interest", BGRImage2_raw);

      //move the opened window
      moveWindow("Select Region of Interest", 20, 20);

      //writing image onto permanent memory
      imwrite(snapshotFileName_BGR, BGRImage2_raw(roi), imageSaveParameters);
    }
  //saving processed BGR image
  else if (*key == '7')
    {
      //Extract the probabilities and labels of objects
      extractClassificationInfo(detectedObjectsInfo, allClassProbabilities, allLabels, allUnknownProbabilities);

      //saving current time into base file name
      saveTimeToString();

      //save cofusion matrix if enabled
      storeProbabilityVectors(snapshotFileName_BGR, allClassProbabilities, allLabels, allUnknownProbabilities);

      //defining the filename for the fused-processed BGR image
      strcpy(snapshotFileName_BGR2, snapshotFileName_BGR);
      strcat(snapshotFileName_BGR2, "-BGR_Fused_Processed.jpg");

      //writing the fused-processed BGR image onto permanent memory
      imwrite(snapshotFileName_BGR2, BGRImageFused_processed, imageSaveParameters);

      //display a message to notify user saving is done
      if (AMALGAMATED_VIEW_ENABLED == false)
	displayStatusBar(FUSED_OUTPUT, "Snapshot taken (Processed Fused Image)", 3000);

      //checking presense of secondary camera's processed frame
      if (ROS_IMAGE2_ENABLE == true)
	{
	  //defining the filename
      	  strcpy(snapshotFileName_BGR2, snapshotFileName_BGR);
      	  strcat(snapshotFileName_BGR2, "-BGR2_Processed.jpg");

      	  //writing image onto permanent memory
      	  imwrite(snapshotFileName_BGR2, BGRImage2_processed, imageSaveParameters);

      	  //display a message to notify user saving is done
      	  if (AMALGAMATED_VIEW_ENABLED == false)
      	    displayStatusBar(SECONDARY_OUTPUT, "Snapshot taken (Processed Secondary Camera)", 3000);
      	}

      /*For amalgamated image if enabled*/
      if (AMALGAMATED_VIEW_ENABLED == true)
	{
	  //defining the filename
	  strcpy(snapshotFileName_Amalgamated, snapshotFileName_BGR);
	  strcat(snapshotFileName_Amalgamated, "-Amalgamated_Processed.jpg");

	  //writing image onto permanent memory
	  imwrite(snapshotFileName_Amalgamated, amalgamated_processed, imageSaveParameters);

	  //display a message to notify user saving is done
	  displayStatusBar(AMALGAMATED_OUTPUT, "Snapshot taken (Processed Views)", 3000);
	}

      //defining the file names
      strcat(snapshotFileName_BGR, "-BGR_Processed.jpg");

      //writing image onto permanent memory
      imwrite(snapshotFileName_BGR, BGRImage_processed, imageSaveParameters);

      //display a message to notify user saving is done
      if (AMALGAMATED_VIEW_ENABLED == false)
	displayStatusBar(MAIN_OUTPUT, "Snapshot taken (Processed Main Camera)", 3000);
    }
}

/***************************/
//The function to save snapshots for calibration
void Snapshot::SaveCalibrationSnapshot(int * key, Mat & BGRImage1, Mat & BGRImage2)
{
  //save main and secondary camera's BGR images
  if (*key == '6')
    {
      //saving current time into base file name
      saveTimeToString();

      //defining the file names
      strcpy(snapshotFileName_BGR2, snapshotFileName_BGR);
      strcat(snapshotFileName_BGR, "-BGR1.jpg");
      strcat(snapshotFileName_BGR2, "-BGR2.jpg");

      //writing images onto permanent memory
      imwrite(snapshotFileName_BGR, BGRImage1, imageSaveParameters);
      imwrite(snapshotFileName_BGR, BGRImage2, imageSaveParameters);

      //display a message to notify user saving is done
      if (AMALGAMATED_VIEW_ENABLED == false)
	displayStatusBar(MAIN_OUTPUT, "Calibration snapshot taken (BGR1 and BGR2)", 3000);
      else
	displayStatusBar(AMALGAMATED_OUTPUT, "Calibration snapshot taken (BGR1 and BGR2)", 3000);
    }

  return;
}

/***************************/
//The function to copy current time into a string
void Snapshot::saveTimeToString()
{
  //save current time to string
  now = time(NULL);
  localTime = localtime(&now);
  strftime(timeString, sizeof(timeString)-1, "%Y-%m-%d_%H:%M:%S", localTime);

  //creating the base file name containing the current time
  strcpy(snapshotFileName_BGR, "");
  strcat(snapshotFileName_BGR, SNAPSHOTS_LOCATION);
  strcat(snapshotFileName_BGR, timeString);

  return;
}

/***************************/
//The function to store probability vectors
void Snapshot::storeProbabilityVectors(char * baseFilename, vector<Mat> * allClassProbabilities, vector<string> * allLabels, vector<double> * allUnknownProbabilities)
{
  if (STORE_PROBABILITY_VECTORS == true)
    {
      //define the file name for main camera
      strcpy(probabilityVectorsFileName, baseFilename);
      strcat(probabilityVectorsFileName, "-Probability_Vectors_MainCamera.txt");

      //open file to be written
      probabilityVectorsFile.open(probabilityVectorsFileName);

      //if not objects detected indicate it in the opened file
      if (allLabels[0].size() == 0)
	probabilityVectorsFile << "No objects detected!"<< endl;

      //write probability vectors along with labels for all objects
      for (counter = 0; counter < allLabels[0].size(); counter++)
	{
	  probabilityVectorsFile << allLabels[0][counter] << ":\n" << allClassProbabilities[0][counter];

	  //write "Unknown Object" probability if Dempster-Shafer decision fusion is enabled
	  if (DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer)
	    probabilityVectorsFile << "\nUnknown-Object Probability:   " << allUnknownProbabilities[0][counter];

	  probabilityVectorsFile << endl << endl;
	}

      //close the file
      probabilityVectorsFile.close();

      //do the same thing for secondary camera if enabled
      if (ROS_IMAGE2_ENABLE == true)
	{
	  //define the file name for main camera
	  strcpy(probabilityVectorsFileName, baseFilename);
	  strcat(probabilityVectorsFileName, "-Probability_Vectors_SecondaryCamera.txt");

	  //open file to be written
	  probabilityVectorsFile.open(probabilityVectorsFileName);

	  //if not objects detected indicate it in the opened file
	  if (allLabels[1].size() == 0)
	    probabilityVectorsFile << "No objects detected!"<< endl;

	  //write probability vectors along with labels for all objects
	  for (counter = 0; counter < allLabels[1].size(); counter++)
	    {
	      probabilityVectorsFile << allLabels[1][counter] << ":\n" << allClassProbabilities[1][counter];

	      //write "Unknown Object" probability if Dempster-Shafer decision fusion is enabled
	      if (DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer)
		probabilityVectorsFile << "\nUnknown-Object Probability:   " << allUnknownProbabilities[1][counter];

	      probabilityVectorsFile << endl << endl;
	    }

	  //close the file
	  probabilityVectorsFile.close();
	}

      /*do the same for the fused view*/
      //define the file name for main camera
      strcpy(probabilityVectorsFileName, baseFilename);
      strcat(probabilityVectorsFileName, "-Probability_Vectors_FusedView.txt");

      //open file to be written
      probabilityVectorsFile.open(probabilityVectorsFileName);

      //if not objects detected indicate it in the opened file
      if (allLabels[2].size() == 0)
	probabilityVectorsFile << "No objects detected!"<< endl;

      //write probability vectors along with labels for all objects
      for (counter = 0; counter < allLabels[2].size(); counter++)
	probabilityVectorsFile << allLabels[2][counter] << ":\n" << allClassProbabilities[2][counter] << endl << endl;

      //close the file
      probabilityVectorsFile.close();
    }

  return;
}

/***************************/
//Function to extract probabilities and labels of objects to prepare their writing on a file, if enabled
void Snapshot::extractClassificationInfo(vector<objectInfo> * input_detectedObjectsInfo, vector<Mat> * output_allClassProbabilities, vector<string> * output_allLabels, vector<double> * output_allUnknownProbabilities)
{
  if (STORE_PROBABILITY_VECTORS == true)
    {
      //Save main/secondary view classification probability vectors and winner labels to be written in a file later, if enabled
      for(cameraCounter = 0; cameraCounter < numberOfCameras; cameraCounter++) //For all the camera views, except the fused view
	{
	  //Clear the output vectors
	  allClassProbabilities[cameraCounter].clear();
	  allLabels[cameraCounter].clear();
	  allUnknownProbabilities[cameraCounter].clear();

	  for(objectCounter = 0; objectCounter < input_detectedObjectsInfo[cameraCounter].size(); objectCounter++) //For all the objects in the view
	    {
	      if (cameraCounter == 0 || (input_detectedObjectsInfo[cameraCounter][objectCounter].matchFound == true))
		{
		  allClassProbabilities[cameraCounter].push_back(input_detectedObjectsInfo[cameraCounter][objectCounter].classProbabilities);
		  allLabels[cameraCounter].push_back(input_detectedObjectsInfo[cameraCounter][objectCounter].label);
		  allUnknownProbabilities[cameraCounter].push_back(input_detectedObjectsInfo[cameraCounter][objectCounter].unknownProbability);
		}
	    }
	}

      //Clear the output vectors
      allClassProbabilities[FUSED_VIEW].clear();
      allLabels[FUSED_VIEW].clear();

      //Save fused classification probability vector and winner label to be written in a file later, if enabled
      for(objectCounter = 0; objectCounter < input_detectedObjectsInfo[FUSED_VIEW].size(); objectCounter++) //For all the objects in the fused view
	{
	  allClassProbabilities[FUSED_VIEW].push_back(input_detectedObjectsInfo[FUSED_VIEW][objectCounter].classProbabilities);
	  allLabels[FUSED_VIEW].push_back(input_detectedObjectsInfo[FUSED_VIEW][objectCounter].label);
	}
    }

  return;
}

