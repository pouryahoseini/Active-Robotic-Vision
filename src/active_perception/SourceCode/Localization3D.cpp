/*
 * Localization3D.cpp
 *
 *  Created on: Apr 25, 2017
 *      Author: pourya
 */

//Header
#include "Localization3D.h"

/**********************************/
//Constructor
Localization3D::Localization3D()
{
  //Subscribing to pointcloud topic
  pointCloud_topic = SIMULATION_WORLD_ENABLE ? ROS_POINTCLOUD_TOPIC_SIMULATION : ROS_POINTCLOUD_TOPIC_REAL_WORLD; //Deciding on the which point cloud topic to use (between the simulation topic and the real world topic)
  sub = nh.subscribe<PointCloud>(pointCloud_topic, 1, & Localization3Dcallback);

  //Subscribing to secondary camera's info topic if it is enabled
  secondary_camera_info_topic = SIMULATION_WORLD_ENABLE ? ROS_IMAGE2_CAMERA_INFO_TOPIC_SIMULATION : ROS_IMAGE2_CAMERA_INFO_TOPIC_REAL_WORLD; //Deciding on the which secondary camera info topic to use (between the simulation topic and the real world topic)
  if(ROS_IMAGE2_ENABLE == true)
    cameraInfoSubscriber = new ROSCameraInfoSubscriber(secondary_camera_info_topic);
}

/**********************************/
//The function to convert 3D camera coordinates in ROS
void Localization3D::transformCameraCoordinate(Point3d & input3DPoint, Point3d & output3DPoint, string sourceFrame, string targetFrame)
{
  //converting the input 3D point to the type tf::Vector3
  point3D_inputFrame.setX(input3DPoint.x);
  point3D_inputFrame.setY(input3DPoint.y);
  point3D_inputFrame.setZ(input3DPoint.z);

  //Get frame transformation
  frameTransform = getFrameTransform(sourceFrame, targetFrame);

  //calculating the converted 3D point
  point3D_outputFrame = frameTransform * point3D_inputFrame;

  //changing the converted 3D point to opencv Vec3f type
  output3DPoint.x = point3D_outputFrame.getX();
  output3DPoint.y = point3D_outputFrame.getY();
  output3DPoint.z = point3D_outputFrame.getZ();

  return;
}

/**********************************/
//The function to get the transformation between two frames
tf::StampedTransform Localization3D::getFrameTransform(string sourceFrame, string targetFrame)
{
  //setting the time to query the server
  ROSTime = ros::Time(0); // ros::Time::now() + ros::Duration(ROS_FRAME_TRANSFORM_TIME_TRAVEL);

  //finding the transformation between the two frames
  try
  {
      // clock_t begin = clock(); //REMOVE #include <ctime>
      frameTransformListener.waitForTransform(targetFrame, sourceFrame, ROSTime, ros::Duration(ROS_FRAME_TRANSFORM_WAIT_TIME));
      frameTransformListener.lookupTransform(targetFrame, sourceFrame, ROSTime, frameTransform);
      // clock_t end = clock(); cout<<endl<<"Elapsed time:   "<<((double(end-begin)/CLOCKS_PER_SEC)*1000)<<"  ms"<<endl;
  }
  catch (tf::TransformException &ex)
  {
      ROS_ERROR("\n\nFailed to transform 3D point between the requested ROS frames\n%s\n", ex.what());
      exit(EXIT_FAILURE);
  }

  return frameTransform;
}

/**********************************/
//The function to convert 3D camera coordinate to 2D pixel coordinate in ROS
void Localization3D::get2DPixelLocation(Point3d & input3DPoint, Point & output2DPoint)
{
  //loading the camera info into the image_geometry::PinholeCameraModel
  if(ROS_IMAGE2_ENABLE == true)
    cameraInfoMessage = cameraInfoSubscriber->getCameraInfoFromROS();

  //loading the camera info into the pinhole camera model
  cameraModel.fromCameraInfo(cameraInfoMessage);

  //projecting the 3D point to the pixel coordinate
  output2DPoint = cameraModel.project3dToPixel(input3DPoint);

  return;
}

/**********************************/
//The function to get the 3D world locations while a pointcloud is being published over the ROS
void Localization3D::get3DWorldPoint(Point inputPoint2D, Point3d & outputPoint3D)
{
  //Rescaling the 2D location to fit in a 512*424 image (size of the undistorted depth image).
  convertedColumn = static_cast<int>(inputPoint2D.x);
  convertedRow = static_cast<int>(inputPoint2D.y);

  //finding the location of the point
  pointIndex = convertedColumn + (convertedRow * Localization3pointsCopy->width);

  //3D coordinates from point cloud using depth value
  if(! Localization3pointsCopy->empty())
    {
      outputPoint3D.x = (float) Localization3pointsCopy->points[pointIndex].x;
      outputPoint3D.y = (float) Localization3pointsCopy->points[pointIndex].y;
      outputPoint3D.z = (float) Localization3pointsCopy->points[pointIndex].z;
    }
  else
    cout<<endl<<"Empty Point Cloud!"<<endl;

  return;
}

/**********************************/
//The callback function necessary to define a ROS subscriber
void Localization3Dcallback(const PointCloud::ConstPtr& msg)
{
  Localization3pointsCopy = msg;

  return;
}
