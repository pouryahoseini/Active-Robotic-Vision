/*
 * VideoRecorder.cpp
 *
 *  Created on: Apr 6, 2017
 *      Author: pourya
 */

//Headers
#include "VideoRecorder.h"

//Constructor
VideoRecorder::VideoRecorder()
{
  recording_original = false;
  recording_processed = false;

  if (RECORDING_ENCODING == "MJPG")
    codec = VideoWriter::fourcc('M','J','P','G');
  else if (RECORDING_ENCODING == "PIM1")
    codec = VideoWriter::fourcc('P','I','M','1');
  else if (RECORDING_ENCODING == "MPEG")
    codec = VideoWriter::fourcc('M','P','E','G');
  else
    {
      fprintf(stderr, "\nVideo encoding not recognized.\nExiting\n\n");
      exit(EXIT_FAILURE);
    }

  return;
}

//The function to record videos
void VideoRecorder::recordVideo( int * key, Mat & bgr_image_original, Mat & bgr_image_processed, Mat & depth_image, Mat & bgr_image2_original, Mat & bgr_image2_processed, Mat & bgr_fused_processed, Mat & amalgamated, int image_height, int image_width )
{
  if (recording_original)
    {
      if ( (* key == 's') || (* key == 'S') )
	{
	  //releasing the video stream
	  video_recorder1.release();
	  video_recorder3.release();

	  //setting recording indicator off
	  recording_original = false;

	  //checking if the secondary camera exists
	  if( ROS_IMAGE2_ENABLE == true )
	    {
	      //releasing the video stream
	      video_recorder4.release();

	      //showing a message to user to notify end of recording
	      if (AMALGAMATED_VIEW_ENABLED == false)
		displayStatusBar(SECONDARY_OUTPUT, "Video recording stopped", 3000);
	    }

	  //showing a message to user to notify end of recording
	  if (AMALGAMATED_VIEW_ENABLED == false)
	    displayStatusBar(MAIN_OUTPUT, "Video recording stopped", 3000);
	}

      //creating a rgb frame out of gray frame
      cvtColor(depth_image, depthImage_colored, CV_GRAY2BGR);

      //writing the current frame onto the permanent memory
      video_recorder1.write(bgr_image_original);
      video_recorder3.write(depthImage_colored);

      //writing the secondary camera's BGR frame onto the permanent memory
      if (ROS_IMAGE2_ENABLE == true)
	video_recorder4.write(bgr_image2_original);
    }
  else
    {
      //Starting to record the original bgr and depth streams (main camera and secondary camera, if it is enabled)
      if (* key == '1')
    	{
	  //saving time as a string
	  saveTimeToString();

	  //defining file address
    	  sprintf(fileAddress, "%s%s_Original_BGR.avi", VIDEO_RECORDING_LOCATION, timeString);
    	  sprintf(fileAddress2, "%s%s_Original_Depth.avi", VIDEO_RECORDING_LOCATION, timeString);

    	  //constructing the video writer class
    	  video_recorder1 = VideoWriter(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);
    	  video_recorder3 = VideoWriter(fileAddress2, codec, RECORDING_FPS, Size(image_width, image_height), true);

    	  //opening the video file
    	  video_recorder1.open(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);
    	  video_recorder3.open(fileAddress2, codec, RECORDING_FPS, Size(image_width, image_height), true);

    	  //checking if secondary camera is enabled
    	  if(ROS_IMAGE2_ENABLE == true)
    	    {
    	      //defining file address
    	      sprintf(fileAddress, "%s%s_Secondary_BGR.avi", VIDEO_RECORDING_LOCATION, timeString);

    	      //constructing the video writer class
    	      video_recorder4 = VideoWriter(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

    	      //opening the video file
    	      video_recorder4.open(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

    	      //showing a message to user to notify start of recording
    	      if (AMALGAMATED_VIEW_ENABLED == false)
    		displayStatusBar(SECONDARY_OUTPUT, "Video recording started (Raw Stream)", 3000);
    	    }

    	  //setting recording indicator on
    	  recording_original = true;

    	  //showing a message to user to notify start of recording
    	  if (AMALGAMATED_VIEW_ENABLED == false)
    	    displayStatusBar(MAIN_OUTPUT, "Video recording started (Raw Stream)", 3000);
    	}
    }


  if (recording_processed)
    {
      if ( (* key == 's') || (* key == 'S') )
      	{
	  //releasing the video stream
	  video_recorder2.release();
	  video_recorder6.release(); // for the fused video stream
	  if (AMALGAMATED_VIEW_ENABLED == true)
	    {
	      video_recorder7.release();
	      displayStatusBar(AMALGAMATED_OUTPUT, "Video recording stopped", 3000);
	    }


	  //setting recording indicator off
	  recording_processed = false;

	  //checking if the secondary camera exists
	  if( ROS_IMAGE2_ENABLE == true )
	    {
	      //releasing the video stream
	      video_recorder5.release();

	      //showing a message to user to notify end of recording
	      if (AMALGAMATED_VIEW_ENABLED == false)
		displayStatusBar(SECONDARY_OUTPUT, "Video recording stopped", 3000);
	    }

	  //showing a message to user to notify end of recording
	  if (AMALGAMATED_VIEW_ENABLED == false)
	    {
	      displayStatusBar(MAIN_OUTPUT, "Video recording stopped", 3000);
	      displayStatusBar(FUSED_OUTPUT, "Video recording stopped", 3000); // for the fused video stream
	    }
      	}

      //writing the current frame onto the permanent memory
      video_recorder2.write(bgr_image_processed);
      video_recorder6.write(bgr_fused_processed); // for the fused video stream
      if (AMALGAMATED_VIEW_ENABLED == true)
	video_recorder7.write(amalgamated);

      //writing the secondary camera's BGR frame onto the permanent memory
      if (ROS_IMAGE2_ENABLE == true)
	video_recorder5.write(bgr_image2_processed);
    }
  else
    {
      //Starting to record the processed bgr stream
      if (* key == '2')
	{
	  //saving time as a string
	  saveTimeToString();

	  //defining file address
	  sprintf(fileAddress, "%s%s_Processed.avi", VIDEO_RECORDING_LOCATION, timeString);

	  //constructing the video writer class
	  video_recorder2 = VideoWriter(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

	  //opening the video file
	  video_recorder2.open(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

	  //setting recording indicator on
	  recording_processed = true;

	  //showing a message to user to notify start of recording
	  if (AMALGAMATED_VIEW_ENABLED == false)
	    displayStatusBar(MAIN_OUTPUT, "Video recording started (Processed Stream)", 3000);

	  //checking if secondary camera is enabled
	  if(ROS_IMAGE2_ENABLE == true)
	    {
	      //defining file address
	      sprintf(fileAddress, "%s%s_Secondary_Processed.avi", VIDEO_RECORDING_LOCATION, timeString);

	      //constructing the video writer class
	      video_recorder5 = VideoWriter(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

	      //opening the video file
	      video_recorder5.open(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

	      //showing a message to user to notify start of recording
	      if (AMALGAMATED_VIEW_ENABLED == false)
		displayStatusBar(SECONDARY_OUTPUT, "Video recording started (Processed Stream)", 3000);
	    }

	  /*For the amalgamated stream if enabled*/
	  if (AMALGAMATED_VIEW_ENABLED == true)
	    {
	      //defining file address
	      sprintf(fileAddress, "%s%s_Amalgamated_Processed.avi", VIDEO_RECORDING_LOCATION, timeString);

	      //constructing the video writer class
	      video_recorder7 = VideoWriter(fileAddress, codec, RECORDING_FPS, Size(GUI_WIDTH, GUI_HEIGHT), true);

	      //opening the video file
	      video_recorder7.open(fileAddress, codec, RECORDING_FPS, Size(GUI_WIDTH, GUI_HEIGHT), true);

	      //showing a message to user to notify start of recording
	      displayStatusBar(AMALGAMATED_OUTPUT, "Video recording started (Processed Stream)", 3000);
	    }

	  /*For the fused video stream*/
	  //defining file address
	  sprintf(fileAddress, "%s%s_Fused_Processed.avi", VIDEO_RECORDING_LOCATION, timeString);

	  //constructing the video writer class
	  video_recorder6 = VideoWriter(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

	  //opening the video file
	  video_recorder6.open(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

	  //showing a message to user to notify start of recording
	  if (AMALGAMATED_VIEW_ENABLED == false)
	    displayStatusBar(FUSED_OUTPUT, "Video recording started (Processed Stream)", 3000);

	}
    }

  //Starting to save both the original and processed streams
  if ( ! recording_original && ! recording_processed && (* key == '3') )
    {
      //saving time as a string
      saveTimeToString();

      //defining file address (for main camera's raw streams)
      sprintf(fileAddress, "%s%s_Original_BGR.avi", VIDEO_RECORDING_LOCATION, timeString);
      sprintf(fileAddress2, "%s%s_Original_Depth.avi", VIDEO_RECORDING_LOCATION, timeString);

      //constructing the video writer class (for main camera's raw streams)
      video_recorder1 = VideoWriter(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);
      video_recorder3 = VideoWriter(fileAddress2, codec, RECORDING_FPS, Size(image_width, image_height), true);

      //opening the video file (for main camera's raw streams)
      video_recorder1.open(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);
      video_recorder3.open(fileAddress2, codec, RECORDING_FPS, Size(image_width, image_height), true);

      //checking if secondary camera is enabled
      if(ROS_IMAGE2_ENABLE == true)
	{
	  //defining file address
	  sprintf(fileAddress, "%s%s_Secondary_BGR.avi", VIDEO_RECORDING_LOCATION, timeString);

	  //constructing the video writer class
	  video_recorder4 = VideoWriter(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

	  //opening the video file
	  video_recorder4.open(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

	  //defining file address
	  sprintf(fileAddress, "%s%s_Secondary_Processed.avi", VIDEO_RECORDING_LOCATION, timeString);

	  //constructing the video writer class
	  video_recorder5 = VideoWriter(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

	  //opening the video file
	  video_recorder5.open(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

	  //showing a message to user to notify start of recording
	  if (AMALGAMATED_VIEW_ENABLED == false)
	    displayStatusBar(SECONDARY_OUTPUT, "Video recording started (Both Raw and Processed)", 3000);
	}

      //defining file address (for main camera's processed stream)
      sprintf(fileAddress, "%s%s_Processed.avi", VIDEO_RECORDING_LOCATION, timeString);

      //constructing the video writer class (for main camera's processed stream)
      video_recorder2 = VideoWriter(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

      //opening the video file (for main camera's processed stream)
      video_recorder2.open(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

      /*For the fused video stream*/
      //defining file address
      sprintf(fileAddress, "%s%s_Fused_Processed.avi", VIDEO_RECORDING_LOCATION, timeString);

      //constructing the video writer class
      video_recorder6 = VideoWriter(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

      //opening the video file
      video_recorder6.open(fileAddress, codec, RECORDING_FPS, Size(image_width, image_height), true);

      /*For amalgamated stream if enabled*/
      if (AMALGAMATED_VIEW_ENABLED == true)
	{
	  //defining file address
	  sprintf(fileAddress, "%s%s_Amalgamated_Processed.avi", VIDEO_RECORDING_LOCATION, timeString);

	  //constructing the video writer class
	  video_recorder7 = VideoWriter(fileAddress, codec, RECORDING_FPS, Size(GUI_WIDTH, GUI_HEIGHT), true);

	  //opening the video file
	  video_recorder7.open(fileAddress, codec, RECORDING_FPS, Size(GUI_WIDTH, GUI_HEIGHT), true);

	  //showing a message to user to notify start of recording
	  displayStatusBar(AMALGAMATED_OUTPUT, "Video recording started (Both Raw and Processed)", 3000);
	}

      //setting recording indicator on
      recording_original = true;
      recording_processed = true;

      //showing a message to user to notify start of recording
      if (AMALGAMATED_VIEW_ENABLED == false)
	{
	  displayStatusBar(MAIN_OUTPUT, "Video recording started (Both Raw and Processed)", 3000);
	  displayStatusBar(FUSED_OUTPUT, "Video recording started (Both Raw and Processed)", 3000); // for the fused video stream
	}
    }

  return;
}

//The function to save current time into a string
void VideoRecorder::saveTimeToString()
{
  //saving current time to a string
  now = time(NULL);
  localTime = localtime(&now);
  strftime(timeString, sizeof(timeString)-1, "%Y-%m-%d_%H:%M:%S", localTime);

  return;
}
