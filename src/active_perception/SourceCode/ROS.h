/*
 * ROS.h
 *
 *  Created on: Mar 7, 2017
 *      Author: pourya
 */

//Guard
#ifndef SRC_ROS_H_
#define SRC_ROS_H_

//Headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <image_transport/transport_hints.h>
#include <ros/callback_queue.h>
#include <cstdlib>
#include <cstdio>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

//Adding support for ROS services and messages
#include "active_vision_msgs/Vision_Message.h"
#include "active_vision_msgs/Vision_Service.h"

//Global definitions
#include "GlobalDefinitions.h"

//Namespaces being used
using namespace std;
using namespace cv;


//The class to subscribe to ROS camera info
class ROSCameraInfoSubscriber
{
public:
  //Constructor
  ROSCameraInfoSubscriber(string topic);

  //The function to retrieve camera info from ROS
  sensor_msgs::CameraInfoConstPtr & getCameraInfoFromROS();

private:
  //The callback function to save camera info coming as ROS messages
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr & cam_info);

  ros::Subscriber cameraInfoSubscriber;
  ros::NodeHandle nh;
  sensor_msgs::CameraInfoConstPtr currentCameraInfoMessage, savedCameraInfoMessage;
};

//The class to subscribe to ROS image stream
class ROSImageSubscriber
{
public:
  //Constructor
  ROSImageSubscriber(const char* topic = "/wide_stereo/right/image_color", cameraTypes imgType = cameraTypes::colorCamera1);

  //The function to retrieve an image from ROS
  Mat getImageFromROS();

private:
  //The callback function to save frames coming as ROS messages
  void imageCallback(const sensor_msgs::ImageConstPtr & msg);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  cv_bridge::CvImagePtr cv_ptr;
  string compression, imageType;
  Mat retrievedImage, retrievedImage2;
};

//The class to publish processed data to ROS
class ROSDataPublisher
{
public:
  //Constructor
  ROSDataPublisher(string frameID);

  //The function to send processed data to ROS
  void dataToROS(int &numberOfObjects, vector<string> & labels, vector<Point3d> & Locations);

private:
  //The callback function to create appopriate responses based on the input requests
  bool publishCallback(active_vision_msgs::Vision_Service::Request & request, active_vision_msgs::Vision_Service::Response & response);

  const char* advertiseAddress = ROS_SERVICE_ADVERTISE_ADDRESS;
  ros::NodeHandle nh_;
  ros::ServiceServer data_pub_;
  active_vision_msgs::Vision_Message objectLocations_message;
  active_vision_msgs::Vision_Service::Response tempResponse, savedResponse;
  vector<geometry_msgs::Point32> Locations3D;
  geometry_msgs::Point32 singleLocation3D;
  string requestedLabel;
  vector<string> allLabels;
  int totalObjects;
  int counter, objectCounter, messageCounter;
};

#endif /* SRC_ROS_H_ */
