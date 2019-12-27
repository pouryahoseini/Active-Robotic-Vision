/*
 * GlobalDefinitions.h
 *
 *  Master Configuration File
 *
 *  Created on: Aug 27, 2016
 *      Author: pourya
 */

//Guard
#ifndef GLOBALDEFINITIONS_H_
#define GLOBALDEFINITIONS_H_

//Headers
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <cmath>

//Namespaces being used
using namespace std;
using namespace cv;

/*****
 * To turn on the training mode, run the program with one of "training", "TRAINING", "train", or "TRAIN" arguments.
 * To take snapshots from raw depth and BGR images, press "0" while the program is working.
*****/

//Run the program in simulation or in real world
extern const bool SIMULATION_WORLD_ENABLE;
extern const string TRAINING_SUBDIRECTORY_NAME[2];

//Robot movement settings
extern const bool ALLOW_ROBOT_MOVEMENT;

//Main camera settings
extern const char * ROS_DEPTH_TOPIC_REAL_WORLD;
extern const char * ROS_DEPTH_TOPIC_SIMULATION;
extern const string ROS_DEPTH_COMPRESSION_REAL_WORLD;
extern const string ROS_DEPTH_COMPRESSION_SIMULATION;
extern const char * ROS_IMAGE1_TOPIC_REAL_WORLD;
extern const char * ROS_IMAGE1_TOPIC_SIMULATION;
extern const string ROS_IMAGE1_COMPRESSION_REAL_WORLD;
extern const string ROS_IMAGE1_COMPRESSION_SIMULATION;
extern const char * ROS_POINTCLOUD_TOPIC_REAL_WORLD;
extern const char * ROS_POINTCLOUD_TOPIC_SIMULATION;
extern const string CAMERA1_FRAME_ID_REAL_WORLD;
extern const string CAMERA1_FRAME_ID_SIMULATION;

//Secondary camera settings
extern const bool ROS_IMAGE2_ENABLE;
extern const char * ROS_IMAGE2_TOPIC_REAL_WORLD;
extern const char * ROS_IMAGE2_TOPIC_SIMULATION;
extern const string ROS_IMAGE2_COMPRESSION_REAL_WORLD;
extern const string ROS_IMAGE2_COMPRESSION_SIMULATION;
extern const string ROS_IMAGE2_CAMERA_INFO_TOPIC_REAL_WORLD;
extern const string ROS_IMAGE2_CAMERA_INFO_TOPIC_SIMULATION;
extern const string CAMERA2_FRAME_ID_REAL_WORLD;
extern const string CAMERA2_FRAME_ID_SIMULATION;

//Outside view camera settings
extern const bool SHOW_OUTSIDE_VIEW_IN_SIMULATION;
extern const char * ROS_OUTSIDE_VIEW_TOPIC;
extern const string ROS_OUTSIDE_VIEW_COMPRESSION;

//Camera types
enum cameraTypes {colorCamera1, colorCamera2, depthCamera, outsideCamera};
const int MAIN_CAMERA = 0, SECONDARY_CAMERA = 1, FUSED_VIEW = 2;

//Run modes
enum runModes {Test, Training_Vision, Training_Locomotion};

//Active vision settings
enum activeVisionModes {None, Deterministic, Intelligent};
extern const volatile activeVisionModes ACTIVE_VISION_MODE;

//Deterministic triangulation planning settings
extern const string TRIANGULATION_PLANNING_BASE_FRAME;
extern const string ROBOT_BASE_FRAME;
extern const float ELBOW_FIXED_VALUE;
extern const float UPPER_ARM_ROLL_VALUE;

//ROS publishing settings
extern const bool PUBLISH_TO_ROS;
extern const char * ROS_NODE_NAME;
extern const char * ROS_SERVICE_ADVERTISE_ADDRESS;

//3D world localization settings (in camera coordinate)
extern const bool ENABLE_3D_WORLD_LOCALIZATION;
extern const double ROS_FRAME_TRANSFORM_WAIT_TIME;

//OpenCV settings
extern const bool ENABLE_OPENCV_CUDA;

//Temporal display settings
extern const int WAIT_KEY_MILLISECOND;

//Specifying the input image sizes
extern const int IMAGE_WIDTH;
extern const int IMAGE_HEIGHT;

/*GUI display setting*/
extern const char * GUI_COMPONENTS_LOCATION;
extern const int GUI_HEIGHT;
extern const int GUI_WIDTH;
//Specify the approximate size of the screen to display the output windows in the proper place
extern const int Screen_Height;
extern const int Screen_Width;

//Output window names
extern const char * MAIN_OUTPUT;
extern const char * SECONDARY_OUTPUT;
extern const char * FUSED_OUTPUT;
extern const char * AMALGAMATED_OUTPUT;
extern const bool AMALGAMATED_VIEW_ENABLED;

//Settings for saving snapshot images
extern const char * SNAPSHOTS_LOCATION;
extern const int SANPSHOT_QUALITY;

//Settings for recording videos
// "MJPG" or "PIM1" or "MPEG"
extern const string RECORDING_ENCODING;
extern const char * VIDEO_RECORDING_LOCATION;
extern const double RECORDING_FPS;

//Preprocessing settings
extern const bool PREPROCESSING_BLUR_TEST_IMAGES;
extern const bool PREPROCESSING_EQUALIZE_HISTOGRAM;

//Detector and classifier type
enum classifierTypes {SVM};
extern const classifierTypes CLASSIFIER_TYPE;
enum detectorTypes {Intensity_Thresholding, Background_Subtraction, Sliding_Window};
extern const detectorTypes DETECTOR_TYPE;

//Training settings
extern const bool DATA_AUGMENTATION;
extern const bool DATA_AUGMENTATION_BACKGROUND_CLASS;
extern const string TRAINING_SOURCE_FOLDER;
extern const string TRAINING_OBJECTS_LIST;
extern const string TRAINING_OBJECTS_EXTENSION;
extern const string BACKGROUND_CLASS_NAME;

//Classifier settings
extern const bool DIFFERENT_PROBABILITY_THRESHOLD_PER_CLASS;
extern const float GLOBAL_OBJECT_PROBABILITY_THRESHOLD;
extern const vector<float> OBJECT_PROBABILITY_THRESHOLD_PER_CLASS; //Threshold probabilities for each class in order of their listing in the training name list file. Note:: The "Background" class probability is trivially included to prevent ordering problems.
extern const char * CLASSIFIER_SOURCE_FOLDER;

//Detector settings
extern const bool DISCARD_CLOSE_IDENTICAL_BOUNDING_BOXES;
extern const int BOUNDING_BOX_PROXIMITY_DISTANCE;

//Object filtering
//Size filtering
extern const bool REJECT_ABNORMAL_OBJECT_SIZES;
extern const int MIN_OBJECT_SIZE;
extern const int MAX_OBJECT_SIZE;
//Depth filtering
extern const bool REJECT_ABNORMAL_DISTANCES;
extern const int MIN_DEPTH;
extern const int MAX_DEPTH;
//Intensity filtering (Too dark areas). Based on the intensity thresholding settings *** Not useful for instensity-based detection, since it already does this kind of rejection
extern const bool REJECT_INTENSITY_BASED;
//Discard the boundary regions of the input images
extern const bool REJECT_INPUT_IMAGE_BOUNDARIES;
extern const float LENGTH_WIDTH_DISCARD_RATIO;

//Intensity thresholding settings
extern const int INTENSITY_THRESHOLD;

//Specifying the ratio of morphological structuring element's dimension to the image dimension
extern const double STRUCTURING_ELEMENT_TO_IMAGE_RATIO;

//Mixture of Gaussian settings
extern const int MOG_VARIABLE_THRESHOLD;
extern const int MOG_HISTORY;
extern const double MOG_LEARNING_RATE_NOMINATOR;

//Sliding window settings
extern const Size SLIDING_WINDOW_SIZE;
extern const int SLIDING_WINDOW_STEP_SIZE;
extern const int SLIDING_WINDOW_INITIAL_MAGNIFICATION;
extern const int IMAGE_PYRAMID_NUMBER_OF_LAYERS;
extern const int NUMBER_OF_WINDOW_SLIMMINGS_EACH_DIMENSION; //For each dimension (i.e. vertical or horizontal), every time the size is cut into half
extern const int NON_MAXIMUM_SUPPRESSION_OVERLAP_THRESHOLD;
extern const int NON_MAXIMUM_SUPPRESSION_OBJECT_NUMBER;

//Matching
enum MatchingTypes {ShortestDistance};
extern const MatchingTypes MATCHING_TYPE;
extern const double STANDARD_DEVIATION_RATIO;
extern const int MAXIMUM_MATCHING_DISTANCE_DENOMINATOR;

//Feature descriptors
//Color histogram
extern const int COLOR_HISTOGRAM_BIN_NUMBER;
//HOG
extern const int REDUCED_HOG_NUMBER_OF_FEATURES;
extern const Size HOG_WINDOWS_SIZE;
extern const Size HOG_BLOCK_SIZE;
extern const Size HOG_BLOCK_STRIDE;
extern const Size HOG_CELL_SIZE;
extern const int HOG_BIN_NUMBER;

//Feature reduction settings
extern const bool ENABLE_FEATURE_REDUCTION;
extern const string FEATURE_REDUCTION_SOURCE_FOLDER;
extern const string FEATURE_REDUCTION_METHOD;
extern const int PCA_TO_LDA_RATIO;

//Python classification settings
extern const char * PYTHON_TRAINING_MODULE_NAME;
extern const char * PYTHON_CLASSIFIER_MODULE_NAME;
extern const char * PYTHON_CLASSIFIER_LOADING_METHOD;
extern const char * PYTHON_FEATURE_CLASSIFIER_TRAINING_CLASS;
extern const char * PYTHON_FEATURE_CLASSIFIER_CLASS;

//Support Vector Machine settings
extern const char * SCIKIT_LEARN_SVM_KERNEL;
extern const char * PYTHON_SVM_CLASSIFIER_METHOD;

//Confidence reckoning
enum CONFIDENCE_MEASURES {firstMaxOverSecondMax, unknownProbability};
extern const CONFIDENCE_MEASURES CONFIDENCE_MEASURE;
extern const double CONFIDENCE_FIRST_MAX_TO_SECOND_MAX_RATIO;
extern const double CONFIDENCE_UNKNOWN_PROBABILITY_THRESHOLD;

//Tracker settings
enum TRACKER_TYPES {KCF, TLD, MIL, MOSSE, Boosting, MedianFlow, GOTURN};
extern const TRACKER_TYPES TRACKER_TYPE;
extern const int TRACKING_INTERVAL; //The number of trackings of objects before another round of detection is executed
extern const int TRACKER_RETRY_AFTER_FAIL;
extern const bool COMPUTE_TRACKING_CERTAINTY;

//Data fusion settings
enum DECISION_FUSION_TYPES {DempsterShafer, Bayesian};
extern const DECISION_FUSION_TYPES DECISION_FUSION_TYPE;
extern const string DEMPSTER_SHAFER_ALL_OBJECTS_CATEGORY_NAME;

//Additional output settings
extern const bool STORE_PROBABILITY_VECTORS;
extern const bool VERBOSE_PRINT_PROBABILITIES;
extern const bool DISPLAY_MID_PROCESS_FRAMES;
extern const bool DISPLAY_TRANSFORMED_CENTROID_IN_SECONDARY_VIEW;
extern const bool DISCARD_UNCERTAIN_FUSED_DETECTIONS;
extern const bool SHOW_UNMATCHED_SECONDARY_DETECTIONS;

//Detection wait (linger) settings (all units are seconds)
extern const int DETECTION_LINGER_TIME;
extern const int HAND_LINGER_TIME_AFTER_DETECTION;
extern const int MOVE_FORWARD_REST_TIME;
extern const int RESPAWNING_LINGER_AFTER_HAND_MOVEMENT;

//Collision avoidance settings
extern const float AVOIDANCE_AREA_CENTER_WIDTH;
extern const float AVOIDANCE_AREA_LENGTH;
extern const float AVOIDANCE_AREA_WIDTH;
extern const float AVOIDANCE_AREA_HEIGHT;
extern const float AVOIDANCE_AREA_TOLERANCE;

//Object placement settings
extern const float PLACEMENT_AREA_LENGTH;
extern const float PLACEMENT_AREA_WIDTH;
extern const int PLACEMENT_AREA_DIVISIONS_PER_AXIS;
extern const float PLACEMENT_AREA_UNIT_HEIGHT;
extern const string OBJECT_DIMENSIONS_FILE_ADDRESS;
extern const string ARTIFICIAL_OBJECTS_GENERIC_NAMES[3];
extern const int ARTIFICIAL_OBJECTS_PER_GENERIC_LABEL[3];
extern const int STRAIGHT_PLACEMENT_COLUMN_GAP;
extern const float OUTSIDE_CAMERA_POSE[3]; // x, y, yaw
extern const bool PRINT_PLACEMENT_AVAILABILITY_MAPS;
extern const bool PRINT_START_FINISH_MESSAGES;

//The structure to hold an object's information
struct objectInfo
{
  string label;
  cv::Mat classProbabilities;
  double winnerProbability, unknownProbability;
  bool isConfirmed, matchFound, isPresent, trackingCertain;
  cv::Point centroid, transformedCentroid;
  int width, height, matchedIndex;
  cv::Rect2d boundingBox;
  cv::Point3d position3D;
  Ptr<cv::Tracker> tracker_ptr; //Smart pointer, so no destructor needed

  //Constructor
  objectInfo() : matchFound(false), isConfirmed(false), isPresent(true), trackingCertain(true), transformedCentroid(Point(0, 0)) {}
};

#endif
