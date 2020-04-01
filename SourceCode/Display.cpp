/*
 * Display.cpp
 *
 *  Created on: Aug 27, 2016
 *      Author: pourya
 */

//Header file
#include "Display.h"


/******************************/
//Constructor definition
Display::Display()
{
  //Identify number of cameras present
  if(ROS_IMAGE2_ENABLE == true)
    numberOfCameras = 2;
  else
    numberOfCameras = 1;

  /*Deciding on the color for every label defined*/
  /*Reading the list of labels from file*/
  numberOfObjects = ObjectListReader(labels);

  //Determine the colors for every object label
  colors.clear();
  for (counter = 0; counter < numberOfObjects; ++counter)
    {
      randomNumber.next();
      colors.push_back(Scalar(randomNumber.uniform(0,255), randomNumber.uniform(0,255), randomNumber.uniform(0,170)));
    }

  //Construct an empty Mat instance to be used later in the displayAtOnce function later
  emptyMat = Mat();
}

/******************************/
//Destructor definition
Display::~Display()
{
  destroyAllWindows();
}

/******************************/
//Function to create named windows
void Display::createWindows()
{
  //creating output window for side by side display of processed main camera, secondary camera, and fused view
  if (AMALGAMATED_VIEW_ENABLED == true)
    {
      //call the function to construct the amalgamated image
      constructAmalgamatedImage();
    }
  else
    {
      namedWindow(MAIN_OUTPUT, CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
      namedWindow(FUSED_OUTPUT, CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);

      //move window to desired location on screen
      moveWindow(MAIN_OUTPUT, Screen_Width / 2, Screen_Height);
      moveWindow(FUSED_OUTPUT, 0, 10);

      //create and move window for showing secondary camera output
      if(ROS_IMAGE2_ENABLE == true)
	{
	  namedWindow(SECONDARY_OUTPUT, CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
	  moveWindow(SECONDARY_OUTPUT, 0, Screen_Height / 2);
      	}
    }

  return;
}

/******************************/
//Function to display output results of the program
void Display::constructOutputImages(vector<objectInfo> detectedObjectsInfo[], Mat output_matchingImage[], Mat output_recognitionOnlyImage[], Mat output_BGRImage[])
{
  //Constructing images for the main view
  for(objectCounter = 0; objectCounter < detectedObjectsInfo[MAIN_CAMERA].size(); objectCounter++) //For all the detected objects in the main view
    {
      //Superimpose results on mid-process frames if enabled
      if (DISPLAY_MID_PROCESS_FRAMES == true)
	{
	  //Set display color without considering confidence
	  displayColor = colorPerLabel(detectedObjectsInfo[MAIN_CAMERA][objectCounter].label);

	  //Create the image with just (matched) colored rectangles (matching evaluation images)
	  rectangle(output_matchingImage[MAIN_CAMERA], detectedObjectsInfo[MAIN_CAMERA][objectCounter].boundingBox, displayColor, 2);

	  //Add labels to the colored rectangles in a new image (recognition only images)
	  addBoundingBox(output_recognitionOnlyImage[MAIN_CAMERA], detectedObjectsInfo[MAIN_CAMERA][objectCounter], displayColor);
	}

      //Set the color for bounding box and texts to red if the object classification is not confident, else set a color based on the detected category of object
      if (detectedObjectsInfo[MAIN_CAMERA][objectCounter].isConfirmed == true)
	displayColor = colorPerLabel(detectedObjectsInfo[MAIN_CAMERA][objectCounter].label);
      else
	displayColor = Scalar(0, 0, 255);

      //Display transformed centroids from the main view in the secondary view, if enabled
      if(DISPLAY_TRANSFORMED_CENTROID_IN_SECONDARY_VIEW && ROS_IMAGE2_ENABLE)
	if(! detectedObjectsInfo[MAIN_CAMERA][objectCounter].isConfirmed) //Only if the object's detection in the main view is uncertain
	  {
	    circle(output_BGRImage[SECONDARY_CAMERA], detectedObjectsInfo[MAIN_CAMERA][objectCounter].transformedCentroid, 5, Scalar(255, 0, 0), -1);

	    //Add the transformation spot to the matching image and recognition only image
	    if(DISPLAY_MID_PROCESS_FRAMES)
	      {
		circle(output_matchingImage[SECONDARY_CAMERA], detectedObjectsInfo[MAIN_CAMERA][objectCounter].transformedCentroid, 5, Scalar(255, 0, 0), -1);
		circle(output_recognitionOnlyImage[SECONDARY_CAMERA], detectedObjectsInfo[MAIN_CAMERA][objectCounter].transformedCentroid, 5, Scalar(255, 0, 0), -1);
	      }
	  }

      //Superimposing the detected object details on the image
      addBoundingBox(output_BGRImage[MAIN_CAMERA], detectedObjectsInfo[MAIN_CAMERA][objectCounter], displayColor);
    }

  //Constructing images for the secondary view, if the secondary view is enabled
  if(ROS_IMAGE2_ENABLE)
    {
      for(objectCounter = 0; objectCounter < detectedObjectsInfo[SECONDARY_CAMERA].size(); objectCounter++) //For all the detected objects in the secondary view
        {
	  //Skip displaying objects that are not matched with any object in the main view
          if((! SHOW_UNMATCHED_SECONDARY_DETECTIONS) && (! detectedObjectsInfo[SECONDARY_CAMERA][objectCounter].matchFound) )
            continue;

          //Superimpose results on mid-process frames if enabled
          if (DISPLAY_MID_PROCESS_FRAMES == true)
            {
              //For matched objects, set color based on their label and add their bounding boxes to the matching image
              if(detectedObjectsInfo[SECONDARY_CAMERA][objectCounter].matchFound)
        	{
        	  //Set display color without considering confidence
        	  displayColor = colorPerLabel(detectedObjectsInfo[SECONDARY_CAMERA][objectCounter].label);

        	  //Set text and line thickness to thick
        	  thickness = THICK_LINE;

        	  //Create the image with just matched colored rectangles (matching evaluation images)
        	  rectangle(output_matchingImage[SECONDARY_CAMERA], detectedObjectsInfo[SECONDARY_CAMERA][objectCounter].boundingBox, displayColor, 2);
        	}
              else //For unmatched objects, set their color to light gray
        	{
        	  displayColor = Scalar(200, 200, 200);

        	  //Set text and line thickness to narrow
        	  thickness = THIN_LINE;
        	}

              //Add labels to the colored rectangles in a new image (recognition only images)
              addBoundingBox(output_recognitionOnlyImage[SECONDARY_CAMERA], detectedObjectsInfo[SECONDARY_CAMERA][objectCounter], displayColor, thickness);
            }

          /*Set the color for bounding box and texts to light gray if the object has no match in the main view
          *and set to red if the object classification is not confident, else set a color based on the detected category of object*/
          if(! detectedObjectsInfo[SECONDARY_CAMERA][objectCounter].matchFound)
            {
              displayColor = Scalar(200, 200, 200);

              //Set text and line thickness to narrow
              thickness = THIN_LINE;
            }
          else
            {
              //Set text and line thickness to thick
              thickness = THICK_LINE;

              if(detectedObjectsInfo[SECONDARY_CAMERA][objectCounter].isConfirmed)
        	displayColor = colorPerLabel(detectedObjectsInfo[SECONDARY_CAMERA][objectCounter].label);
              else
        	displayColor = Scalar(0, 0, 255);
            }

          //Superimposing the detected object details on the image
          addBoundingBox(output_BGRImage[SECONDARY_CAMERA], detectedObjectsInfo[SECONDARY_CAMERA][objectCounter], displayColor, thickness);
        }
    }

  //Constructing images for the fused view
  for(objectCounter = 0; objectCounter < detectedObjectsInfo[FUSED_VIEW].size(); objectCounter++) //For all the detected objects in the fused view
    {
      //Skip displaying for the case of objects with classifications that are not certain, if it is enabled
      if((DISCARD_UNCERTAIN_FUSED_DETECTIONS && (! detectedObjectsInfo[FUSED_VIEW][objectCounter].isConfirmed)) || (detectedObjectsInfo[FUSED_VIEW][objectCounter].label == BACKGROUND_CLASS_NAME))
	continue;

      //Set the color for bounding box and texts to a color based on the detected category of object
      displayColor = colorPerLabel(detectedObjectsInfo[FUSED_VIEW][objectCounter].label);

      //Superimposing the fused object details on the fused view image
      addBoundingBox(output_BGRImage[FUSED_VIEW], detectedObjectsInfo[FUSED_VIEW][objectCounter], displayColor);
    }

  return;
}

/******************************/
//Function to print mid-process info about the detected objects
void Display::printOutputInfo(vector<objectInfo> detectedObjectsInfo[])
{
  if (VERBOSE_PRINT_PROBABILITIES == true)
    {
      for(objectCounter = 0; objectCounter < detectedObjectsInfo[FUSED_VIEW].size(); objectCounter++) //For all the detected objects in the fused view
	{
	  //Determine the matched object in the secondary view
          matchedIndex = detectedObjectsInfo[MAIN_CAMERA][objectCounter].matchedIndex;

          //Print classification probabilities in the terminal if enabled
          if(DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::DempsterShafer)
            cout << "\n**************\n- Main camera class probabilities + All-Objects class probability:\n" << detectedObjectsInfo[MAIN_CAMERA][objectCounter].classProbabilities << "  *****  " << detectedObjectsInfo[MAIN_CAMERA][objectCounter].unknownProbability << "\n- Secondary camera class probabilities + All-Objects class probability:\n" << detectedObjectsInfo[SECONDARY_CAMERA][matchedIndex].classProbabilities << "  ****  " << detectedObjectsInfo[SECONDARY_CAMERA][matchedIndex].unknownProbability << "\n- Fused view class probabilities:\n" << detectedObjectsInfo[FUSED_VIEW][objectCounter].classProbabilities << endl;
          else if(DECISION_FUSION_TYPE == DECISION_FUSION_TYPES::Bayesian)
            cout << "\n**************\n- Main camera class probabilities:\n" << detectedObjectsInfo[MAIN_CAMERA][objectCounter].classProbabilities << "\n- Secondary camera class probabilities:\n" << detectedObjectsInfo[SECONDARY_CAMERA][matchedIndex].classProbabilities << "\n- Fused view class probabilities:\n" << detectedObjectsInfo[FUSED_VIEW][objectCounter].classProbabilities << endl;
        }
    }

  return;
}

/******************************/
//Function to add a bounding box to an image
void Display::addBoundingBox(Mat & manipulatedImage, objectInfo & object_info, Scalar inputColor, int inputThickness)
{
  rectangle(manipulatedImage, object_info.boundingBox, inputColor, inputThickness);
  putText(manipulatedImage, object_info.label, object_info.centroid, FONT_HERSHEY_COMPLEX_SMALL, 1, inputColor, inputThickness);
  putText(manipulatedImage, to_string(object_info.winnerProbability), object_info.centroid - Point(25,25), FONT_HERSHEY_COMPLEX_SMALL, 0.8, inputColor, ((inputThickness / 4.0f) + 0.5));

  return;
}

/******************************/
//Function to display amalgamated/separated frames and mid process frames at once
Mat & Display::displayAtOnce(Mat outputImage[3], Mat recognitionOnlyImage[2], Mat matchingImage[2], Timer * timer, Mat & outsideView)
{
  //Show mid-process frames if enabled
  if (DISPLAY_MID_PROCESS_FRAMES == true)
	displayMidProcess(recognitionOnlyImage, matchingImage);

  /*Displaying output frames (i.e. processed main camera, secondary camera (if exists), and fused view)*/
  if (AMALGAMATED_VIEW_ENABLED == true) //Check to show an amalgamated view or separate views
    {
      //Display amalgamated frames
      return displayAmalgamatedFrame(outputImage[MAIN_CAMERA], outputImage[SECONDARY_CAMERA], outputImage[FUSED_VIEW], timer, outsideView);
    }
  else
    {
      //Display outputs in separate windows
      displaySeparatedFrames(outputImage[MAIN_CAMERA], outputImage[SECONDARY_CAMERA], outputImage[FUSED_VIEW], timer);

      return emptyMat;
    }
}

/******************************/
//Threaded function to display mid-process images
void Display::displayMidProcess(Mat recognitionOnlyImage[2], Mat matchingImage[2])
{
  //Show object labels without specific coloring for confidence check.
  imshow("Object Recognition - Camera #1", recognitionOnlyImage[MAIN_CAMERA]);

  if(ROS_IMAGE2_ENABLE)
    {
      imshow("Object Recognition - Camera #2", recognitionOnlyImage[SECONDARY_CAMERA]);

      //Show matchings without object labels
      imshow("Object Matching - Camera #1", matchingImage[MAIN_CAMERA]);
      imshow("Object Matching - Camera #2", matchingImage[SECONDARY_CAMERA]);
    }

  return;
}

/******************************/
//Threaded function to display amalgamated output
Mat & Display::displayAmalgamatedFrame(Mat & inputMainCameraFrame, Mat & inputSecondaryCameraFrame, Mat & inputFusedFrame, Timer * timer, Mat & outsideView)
{
  /*Building the big amalgamated image*/
  //Resize images to be displayed if necessary
  resizeIfNecessary(inputMainCameraFrame, inputSecondaryCameraFrame, inputFusedFrame, outsideView);

  //Copying the three output images on the big image
  inputFusedFrame_confirmedSize.copyTo(amalgamatedImage(Rect(Point(40, 5), inputFusedFrame_confirmedSize.size())));
  inputMainCameraFrame_confirmedSize.copyTo(amalgamatedImage(Rect(Point(40, (GUI_HEIGHT / 2) + 10), inputMainCameraFrame_confirmedSize.size())));
  inputSecondaryCameraFrame_confirmedSize.copyTo(amalgamatedImage(Rect(Point((GUI_WIDTH / 2) + 20, (GUI_HEIGHT / 2) + 10), inputSecondaryCameraFrame_confirmedSize.size())));
  if(SIMULATION_WORLD_ENABLE && SHOW_OUTSIDE_VIEW_IN_SIMULATION)
    outsideView_confirmedSize.copyTo(amalgamatedImage(Rect(Point((GUI_WIDTH / 2) + 20, 5), outsideView_confirmedSize.size())));

  //Showing the amalgamated image
  imshow(AMALGAMATED_OUTPUT, amalgamatedImage);

  //Display frame rate on the amalgamated output
  timer->overlayFrameRate_CameraStream(AMALGAMATED_OUTPUT);

  return amalgamatedImage;
}

/******************************/
//Threaded function to display separate outputs (i.e. main camera, secondary camera, and fused view)
void Display::displaySeparatedFrames(Mat & inputMainCameraFrame, Mat & inputSecondaryCameraFrame, Mat & inputFusedFrame, Timer * timer)
{
  //Show main and fused outputs
  imshow(MAIN_OUTPUT, inputMainCameraFrame);
  imshow(FUSED_OUTPUT, inputFusedFrame);

  //Display frame rate on separate outputs
  timer->overlayFrameRate_CameraStream(MAIN_OUTPUT);
  timer->overlayFrameRate_CameraStream(FUSED_OUTPUT);

  //Display the current frame and overlaying frame rate for secondary camera if it is enabled
  if(ROS_IMAGE2_ENABLE == true)
    {
      imshow(SECONDARY_OUTPUT, inputSecondaryCameraFrame);
      timer->overlayFrameRate_CameraStream(SECONDARY_OUTPUT);
    }

  return;
}

/******************************/
//The function to resize images to be displayed if necessary
void Display::resizeIfNecessary(Mat & inputMainCameraFrame, Mat & inputSecondaryCameraFrame, Mat & inputFusedFrame, Mat & outsideView)
{
  //Resize main camera's image if necessary
  if ((inputMainCameraFrame.rows > ((GUI_HEIGHT / 2) - 10)) || (inputMainCameraFrame.cols > ((GUI_WIDTH / 2) - 60)))
    resize(inputMainCameraFrame, inputMainCameraFrame_confirmedSize, Size((GUI_WIDTH / 2) - 60, (GUI_HEIGHT / 2) - 10));
  else
    inputMainCameraFrame_confirmedSize = inputMainCameraFrame;

  //Resize secondary camera's image if necessary
  if ((inputSecondaryCameraFrame.rows > ((GUI_HEIGHT / 2) - 10)) || (inputSecondaryCameraFrame.cols > ((GUI_WIDTH / 2) - 60)))
    resize(inputSecondaryCameraFrame, inputSecondaryCameraFrame_confirmedSize, Size((GUI_WIDTH / 2) - 60, (GUI_HEIGHT / 2) - 10));
  else
    inputSecondaryCameraFrame_confirmedSize = inputSecondaryCameraFrame;

  //Resize fused image if necessary
  if ((inputFusedFrame.rows > ((GUI_HEIGHT / 2) - 10)) || (inputFusedFrame.cols > ((GUI_WIDTH / 2) - 60)))
    resize(inputFusedFrame, inputFusedFrame_confirmedSize, Size((GUI_WIDTH / 2) - 60, (GUI_HEIGHT / 2) - 10));
  else
    inputFusedFrame_confirmedSize = inputFusedFrame;

  //Resize outside view image if necessary
  if ((outsideView.rows > ((GUI_HEIGHT / 2) - 10)) || (outsideView.cols > ((GUI_WIDTH / 2) - 60)))
    resize(outsideView, outsideView_confirmedSize, Size((GUI_WIDTH / 2) - 60, (GUI_HEIGHT / 2) - 10));
  else
    outsideView_confirmedSize = outsideView;

  return;
}

/******************************/
//The function to decide a color per label
Scalar Display::colorPerLabel(string & inputLabel)
{
  //find the location of the input label in the list of labels
  labelCounter = 0;
  while (! (inputLabel == labels[labelCounter]))
    labelCounter++;

  //return the color corresponding to the input label
  return colors[labelCounter];
}

/******************************/
//The function to display fixed interface on the amalgamated window
void Display::constructAmalgamatedImage()
{
  //create window
  namedWindow(AMALGAMATED_OUTPUT, CV_WINDOW_AUTOSIZE | CV_GUI_EXPANDED);

  //move window to desired location on screen
  moveWindow(AMALGAMATED_OUTPUT, 250, 10);

  //set background color
  backgroundColor = Scalar(210, 210, 210);

  //constructing the big image
  amalgamatedImage = Mat(Size(GUI_WIDTH, GUI_HEIGHT), CV_8UC3, backgroundColor);

  /*adding labels for every view on the window*/
  //create a horizontal manipulation image, put the text and rotate that to vertical orientation, and copy it on the amalgamated image
  //fused view
  manipulationImage = Mat(Size(GUI_HEIGHT / 4, GUI_WIDTH / 30), CV_8UC3, backgroundColor);
  putText(manipulationImage, "Actively Fused View", Point(1, GUI_WIDTH / 50), FONT_HERSHEY_COMPLEX, 0.7, Scalar(0, 0, 0), 2);
  rotate(manipulationImage, manipulationImage, ROTATE_90_COUNTERCLOCKWISE);
  manipulationImage.copyTo(amalgamatedImage(Rect(Point(1, GUI_HEIGHT / 10), manipulationImage.size())));

  //main camera
  manipulationImage = Mat(Size(GUI_HEIGHT / 4, GUI_WIDTH / 30), CV_8UC3, backgroundColor);
  putText(manipulationImage, "Main Camera View", Point(1, GUI_WIDTH / 50), FONT_HERSHEY_COMPLEX, 0.7, Scalar(0, 0, 0), 2);
  rotate(manipulationImage, manipulationImage, ROTATE_90_COUNTERCLOCKWISE);
  manipulationImage.copyTo(amalgamatedImage(Rect(Point(1, (int) (GUI_HEIGHT * 0.62)), manipulationImage.size())));

  //secondary camera
  manipulationImage = Mat(Size(GUI_HEIGHT / 3, GUI_WIDTH / 30), CV_8UC3, backgroundColor);
  putText(manipulationImage, "Secondary Camera View", Point(1, GUI_WIDTH / 50), FONT_HERSHEY_COMPLEX, 0.7, Scalar(0, 0, 0), 2);
  rotate(manipulationImage, manipulationImage, ROTATE_90_COUNTERCLOCKWISE);
  manipulationImage.copyTo(amalgamatedImage(Rect(Point(static_cast<int>(GUI_WIDTH - (GUI_WIDTH / 30)), (int) (GUI_HEIGHT * 0.57)), manipulationImage.size())));

  //outside view camera
  if(SIMULATION_WORLD_ENABLE && SHOW_OUTSIDE_VIEW_IN_SIMULATION)
    {
      manipulationImage = Mat(Size(GUI_HEIGHT / 4, GUI_WIDTH / 30), CV_8UC3, backgroundColor);
      putText(manipulationImage, "Outside View", Point(1, GUI_WIDTH / 50), FONT_HERSHEY_COMPLEX, 0.7, Scalar(0, 0, 0), 2);
      rotate(manipulationImage, manipulationImage, ROTATE_90_COUNTERCLOCKWISE);
      manipulationImage.copyTo(amalgamatedImage(Rect(Point(static_cast<int>(GUI_WIDTH - (GUI_WIDTH / 30)), GUI_HEIGHT / 12), manipulationImage.size())));

    }
  /*adding fixed images on the aggregated image to be displayed*/
  else //adding University of Nevada, Reno's logo
    {
      readImage = imread((string(GUI_COMPONENTS_LOCATION) + string("UNR-Logo.png")));
      readImage.copyTo(amalgamatedImage(Rect(Point((GUI_WIDTH / 2) + 120, 150), readImage.size())));
    }

  return;
}

