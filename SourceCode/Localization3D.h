/*
 * Localization3D.h
 *
 *  Created on: Apr 25, 2017
 *      Author: pourya
 */

//Guard
#ifndef SRC_LOCALIZATION3D_H_
#define SRC_LOCALIZATION3D_H_

//Headers
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <ctime>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <image_geometry/pinhole_camera_model.h>

#include "ROS.h"

//typedefs
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//Global definitions
#include "GlobalDefinitions.h"

//Namespaces being used
using namespace std;
using namespace cv;

//The class to convert pixel coordinate to camera coordinate
class Localization3D
{
public:
  //Constructor
  Localization3D();

  //The function to convert 3D camera coordinates in ROS
  void transformCameraCoordinate(Point3d & input3DPoint, Point3d & output3DPoint, string sourceFrame, string targetFrame);

  //The function to get the transformation between two frames
  tf::StampedTransform getFrameTransform(string sourceFrame, string targetFrame);

  //The function to convert 3D camera coordinate to 2D pixel coordinate in ROS
  void get2DPixelLocation(Point3d & input3DPoint, Point & output2DPoint);

  //The function to get the 3D world locations while a pointcloud is being published over the ROS
  void get3DWorldPoint(Point inputPoint2D, Point3d & outputPoint3D);

private:
  //Declaring as friend the callback function necessary to define a ROS subscriber
  friend void Localization3Dcallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  int convertedRow, convertedColumn, counter, pointIndex, arrayPosX, arrayPosY, arrayPosZ;
  Mat resizedDepthImage;
  float X, Y, Z;
  ros::NodeHandle nh;
  ros::Subscriber sub;
  tf::Vector3 point3D_inputFrame, point3D_outputFrame;
  tf::TransformListener frameTransformListener;
  tf::StampedTransform frameTransform;
  ros::Time ROSTime;
  sensor_msgs::CameraInfoConstPtr cameraInfoMessage;
  image_geometry::PinholeCameraModel cameraModel;
  ROSCameraInfoSubscriber * cameraInfoSubscriber;
  string secondary_camera_info_topic;
  const char * pointCloud_topic;
};

PointCloud::ConstPtr Localization3pointsCopy(new PointCloud);
void Localization3Dcallback(const PointCloud::ConstPtr& msg);

/*
sensor_msgs::PointCloud2ConstPtr Localization3DmsgCopy;

//callback function for subscription to ROS pointcloud
//static void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
void Localization3Dcallback(const sensor_msgs::PointCloud2ConstPtr& msg);
*/

#endif /* SRC_LOCALIZATION3D_H_ */
