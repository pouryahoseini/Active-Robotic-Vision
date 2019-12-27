##Commented, because of ROS CMake file## cmake_minimum_required(VERSION 2.6)
##Commented, because of ROS CMake file## project(Active_Perception)

set(CMAKE_BUILD_TYPE Debug)

#Giving the true path to the OpenCV library (and override ROS's version of OpenCV)
#find_package(OpenCV 3.1 REQUIRED)
set(OpenCV_INCLUDE_DIR /usr/local/include)
set(OpenCV_LIB_DIR /usr/local/lib)
set(OpenCV_LIBS opencv_cudabgsegm opencv_cudaobjdetect opencv_superres opencv_cudastereo opencv_stitching opencv_cudafeatures2d opencv_videostab opencv_cudaoptflow opencv_cudalegacy opencv_cudawarping opencv_cudacodec opencv_stereo opencv_xphoto opencv_aruco opencv_saliency opencv_bioinspired opencv_hfs opencv_xobjdetect opencv_dnn_objdetect opencv_cvv opencv_rgbd opencv_reg opencv_surface_matching opencv_dpm opencv_structured_light opencv_viz opencv_tracking opencv_ccalib opencv_datasets opencv_text opencv_dnn opencv_img_hash opencv_optflow opencv_ximgproc opencv_freetype opencv_fuzzy opencv_xfeatures2d opencv_calib3d opencv_shape opencv_ml opencv_face opencv_photo opencv_cudaimgproc opencv_cudafilters opencv_video opencv_objdetect opencv_cudaarithm opencv_phase_unwrapping opencv_hdf opencv_line_descriptor opencv_features2d opencv_highgui opencv_videoio opencv_imgcodecs opencv_imgproc opencv_flann opencv_core opencv_cudev)
#set(OpenCV_LIBS opencv_aruco opencv_bgsegm opencv_bioinspired opencv_calib3d opencv_ccalib opencv_core opencv_cvv opencv_datasets opencv_dnn opencv_dpm opencv_face opencv_features2d opencv_flann opencv_fuzzy opencv_hdf opencv_highgui opencv_imgcodecs opencv_imgproc opencv_line_descriptor opencv_ml opencv_objdetect opencv_optflow opencv_photo opencv_plot opencv_reg opencv_rgbd opencv_saliency opencv_shape opencv_stereo opencv_stitching opencv_structured_light opencv_superres opencv_surface_matching opencv_text opencv_tracking opencv_videoio opencv_video opencv_videostab opencv_viz opencv_xfeatures2d opencv_ximgproc opencv_xobjdetect opencv_xphoto)

#Forcing to use C++11 standard
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Explicitly link to OpenCL (in case of CUDA installed)
FIND_PACKAGE(OpenCL QUIET)
IF(OpenCL_FOUND)
	SET(LIBRARIES
		${OpenCL_LIBRARIES}
		${LIBRARIES}
	)
ENDIF(OpenCL_FOUND)

#Finding Python and Boost.Python libraries
find_package(PythonInterp 2.7 REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)

SET(BOOST_MIN_VERSION "1.58.0")
FIND_PACKAGE(Boost ${BOOST_MIN_VERSION} REQUIRED)

IF(Boost_FOUND)
  #SET(Boost_USE_STATIC_LIBS ON)
  SET(Boost_USE_MULTITHREADED ON)
  #SET(Boost_USE_STATIC_RUNTIME ON)
  FIND_PACKAGE(Boost COMPONENTS python)
ELSEIF(NOT Boost_FOUND)
  MESSAGE(FATAL_ERROR "Unable to find correct Boost version. Did you set BOOST_ROOT?")
ENDIF()

#setting the boost.python and bosst.python's numpy manually. Because of their bug
set(Boost_LIBRARIES "/usr/lib/x86_64-linux-gnu/libboost_python-py27.so")
set(Boost_Numpy_LIBRARIES "/usr/local/lib64/libboost_numpy.so") #"/usr/local/lib/libboost_numpy3.so"

#Providing the location of include and library directories
include_directories( ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} )
link_directories( ${Boost_Numpy_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIB_DIR} ${OpenCL_LIBRARIES})

#Compiling files except main.cpp (for each file an add_library instance is needed)
add_library(GlobalDefinitions  GlobalDefinitions.cpp GlobalDefinitions.h)
add_library(Timer Timer.cpp Timer.h)
add_library(Display Display.cpp Display.h)
add_library(Termination Termination.cpp Termination.h)
add_library(Morphology Morphology.cpp Morphology.h)
add_library(ROS ROS.cpp ROS.h)
add_library(VisionTraining VisionTraining.cpp VisionTraining.h)
add_library(Snapshot Snapshot.cpp Snapshot.h)
add_library(Classification Classification.cpp Classification.h)
add_library(Detection Detection.cpp Detection.h)
add_library(FeatureDescriptor FeatureDescriptor.cpp FeatureDescriptor.h)
add_library(VideoRecorder VideoRecorder.cpp VideoRecorder.h)
add_library(FeatureReduction FeatureReduction.cpp FeatureReduction.h)
add_library(Localization3D Localization3D.cpp Localization3D.h)
add_library(PythonCppConvertor PythonCppConvertor.cpp PythonCppConvertor.h)
add_library(ConfidenceCheck ConfidenceCheck.cpp ConfidenceCheck.h)
add_library(Matching Matching.cpp Matching.h)
add_library(DataFusion DataFusion.cpp DataFusion.h)
add_library(SimulationWorld SimulationWorld.cpp SimulationWorld.h)
add_library(RobotMovement RobotMovement.cpp RobotMovement.h)
add_library(Preprocessing Preprocessing.cpp Preprocessing.h)
add_library(TrackingManagement TrackingManagement.cpp TrackingManagement.h)
add_library(PlannerTraining PlannerTraining.cpp PlannerTraining.h)
add_library(Planning Planning.cpp Planning.h)

#Compiling main.cpp and linking other compiled objects to it (pthread is used in the code for Kinect1)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries( ${PROJECT_NAME}
	GlobalDefinitions Timer Display Termination Morphology RobotMovement 
    ROS VisionTraining Snapshot Classification FeatureDescriptor VideoRecorder FeatureReduction 
    Localization3D PythonCppConvertor ConfidenceCheck Matching DataFusion SimulationWorld 
    Detection Preprocessing TrackingManagement PlannerTraining Planning 
    ${OpenCV_LIBS} ${OpenCL_LIBRARIES} pthread ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} 
    ${Boost_Numpy_LIBRARIES} ) 
    # ${catkin_LIBRARIES} is just added for integrating ROS specified in the parent CMakeLists.txt


