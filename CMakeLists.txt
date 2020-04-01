cmake_minimum_required(VERSION 2.8.3)

project(active_perception)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs image_transport cv_bridge) #This line is modified by Pourya Hoseini

               #######
####################################
#   My additions: Pourya Hoseini   #

find_package( catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp sensor_msgs std_msgs 
# message_generation
 tf image_geometry rospy moveit_ros_planning_interface moveit_msgs 
 active_vision_msgs)

#########################
#PointCloud library setup
find_package( PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#########################

#set the include directory to the generated header files for messaging over ROS
set(MESSAGING_INCLUDE_DIRECTORY ./devel/include)

# add_service_files(
#  FILES
#  Vision_Service.srv
#)

#add_message_files(
#  FILES
#  Vision_Message.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

catkin_package( CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs 
#message_runtime 
tf image_geometry rospy)

include_directories(${catkin_INCLUDE_DIRS} ${MESSAGING_INCLUDE_DIRECTORY})

add_subdirectory(SourceCode)

#target_link_libraries(${CURRENT_CMAKE_DIR}/SourceCode/${PROJECT_NAME} ${catkin_LIBRARIES})

