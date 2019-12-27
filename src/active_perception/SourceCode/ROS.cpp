/*
 * ROS.cpp
 *
 *  Created on: Mar 7, 2017
 *      Author: pourya
 */

//Header file
#include "ROS.h"
/*************************** ROSCameraInfoSubscriber **********************************/
/**************************************************/
//The constructor to subscribe to ROS camera information
ROSCameraInfoSubscriber::ROSCameraInfoSubscriber(string topic)
{
  //Subscribing to the specified camera info topic
  cameraInfoSubscriber = nh.subscribe(topic, 1, & ROSCameraInfoSubscriber::cameraInfoCallback, this);
}

/**************************************************/
//The callback function to save camera info coming as ROS messages
void ROSCameraInfoSubscriber::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr & cam_info)
{
  //saving the camera info message
  currentCameraInfoMessage = cam_info;

  return;
}

/**************************************************/
//The function to retrieve camera info from ROS
sensor_msgs::CameraInfoConstPtr & ROSCameraInfoSubscriber::getCameraInfoFromROS()
{
  //saving a copy of the camera info message
  savedCameraInfoMessage = currentCameraInfoMessage;

  return savedCameraInfoMessage;
}

/*************************** ROSImageSubscriber **********************************/
/**************************************************/
//The constructor to subscribe to ROS images
ROSImageSubscriber::ROSImageSubscriber(const char* topic, cameraTypes imgType) : it_(nh_)
{
  //Determining the matrix and compression type according to the color format
  if(imgType == cameraTypes::colorCamera1)
  {
      imageType = sensor_msgs::image_encodings::BGR8;
      compression = SIMULATION_WORLD_ENABLE ? ROS_IMAGE1_COMPRESSION_SIMULATION : ROS_IMAGE1_COMPRESSION_REAL_WORLD;
  }
  else if(imgType == cameraTypes::colorCamera2)
    {
      //Return if the secondary camera is not enabled
      if(! ROS_IMAGE2_ENABLE)
	return;

      imageType = sensor_msgs::image_encodings::TYPE_8UC3;
      compression = SIMULATION_WORLD_ENABLE ? ROS_IMAGE2_COMPRESSION_SIMULATION : ROS_IMAGE2_COMPRESSION_REAL_WORLD;
    }
  else if(imgType == cameraTypes::depthCamera)
    {
      imageType = sensor_msgs::image_encodings::TYPE_32FC1;
      compression = SIMULATION_WORLD_ENABLE ? ROS_DEPTH_COMPRESSION_SIMULATION : ROS_DEPTH_COMPRESSION_REAL_WORLD;
    }
  else if(imgType == cameraTypes::outsideCamera)
    {
      imageType = sensor_msgs::image_encodings::BGR8;
      compression = ROS_OUTSIDE_VIEW_COMPRESSION;
    }

  //Subscribe to input video feed and publish output video feed
  try
  {
      image_sub_ = it_.subscribe(topic, 1, & ROSImageSubscriber::imageCallback, this, image_transport::TransportHints(compression));
  }
  catch(...)
  {
      fprintf(stderr, "\n\nError in subscribing to the following camera topic:\n%s\n\n", topic);
      exit(EXIT_FAILURE);
  }

}

/**************************************************/
//The callback function to save frames coming as ros messages
void ROSImageSubscriber::imageCallback(const sensor_msgs::ImageConstPtr & msg)
{

  try
  {
      cv_ptr = cv_bridge::toCvCopy(msg, imageType);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  //saving the retrieved image
  retrievedImage = cv_ptr->image;
/*
  if (imageType == sensor_msgs::image_encodings::TYPE_8UC3){
  //cv_ptr->image *=50; cv_ptr->image.convertTo(cv_ptr->image, CV_8U);
  imshow("lmk", cv_ptr->image); waitKey(1);}*/
}

/**************************************************/
//The function to retrieve an image from ROS
Mat ROSImageSubscriber::getImageFromROS()
{
  retrievedImage2 = retrievedImage.clone();
  return retrievedImage2;
}

/*************************** ROSDataPublisher **********************************/
/**************************************************/
//The constructor to publish images to ROS
ROSDataPublisher::ROSDataPublisher(string frameID)
{
  objectLocations_message.Frameid = frameID;

  if(PUBLISH_TO_ROS)
    data_pub_ = nh_.advertiseService(advertiseAddress, &ROSDataPublisher::publishCallback, this);
}

/**************************************************/
//The callback function to create appopriate responses based on the input requests
bool ROSDataPublisher::publishCallback(active_vision_msgs::Vision_Service::Request & request, active_vision_msgs::Vision_Service::Response & response)
{
  //Clear the response message
  tempResponse.object_location.clear();

  //saving the requested label
  requestedLabel = request.Label;

  //Finding the object location
  objectLocations_message.Found = false;
  for(counter = 0; counter < totalObjects; counter++)
    {
      if(requestedLabel == allLabels[counter])
	{
	  objectLocations_message.Found = true;
	  objectLocations_message.Pos = Locations3D[counter];

	  //updating the temporary response message
	  tempResponse.object_location.push_back(objectLocations_message);
	}
    }

  //Update the response with the last message with the "object found" boolean variables set to false if no objects are found
  if(objectLocations_message.Found == true)
    response = (savedResponse = tempResponse);
  else
    {
      response = savedResponse;
      for(messageCounter = 0; messageCounter < response.object_location.size(); messageCounter++)
	response.object_location[messageCounter].Found = false;
    }

  return true;
}

/**************************************************/
//The function to send processed data to ROS
void ROSDataPublisher::dataToROS(int &numberOfObjects, vector<string> & labels, vector<Point3d> & Locations)
{
  Locations3D.clear();

  for(objectCounter = 0; objectCounter < numberOfObjects; objectCounter++)
    {
      singleLocation3D.x = Locations[objectCounter].x;
      singleLocation3D.y = Locations[objectCounter].y;
      singleLocation3D.z = Locations[objectCounter].z;
      Locations3D.push_back(singleLocation3D);
    }

  allLabels = labels;

  totalObjects = numberOfObjects;

  return;
}
