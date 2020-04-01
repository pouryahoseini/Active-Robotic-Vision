/*
 * GlobalDefinitions.cpp
 *
 *  Created on: Jan 13, 2018
 *      Author: pourya
 */

//Header file
#include "GlobalDefinitions.h"

//Run the program in simulation or in real world
const bool SIMULATION_WORLD_ENABLE = true;
const string TRAINING_SUBDIRECTORY_NAME[2] = {"RealWorld", "Simulation"};

//Robot movement settings
const bool ALLOW_ROBOT_MOVEMENT = true;

//Main camera settings
const char * ROS_DEPTH_TOPIC_REAL_WORLD = "/kinect_head/depth/image_rect_raw"; // "/kinect2/qhd/image_depth_rect"; // "/kinect_head/depth_registered/sw_registered/image_rect_raw"
const char * ROS_DEPTH_TOPIC_SIMULATION = "/head_mount_kinect/depth/image_raw";
const string ROS_DEPTH_COMPRESSION_REAL_WORLD = "compressedDepth"; // "compressed";
const string ROS_DEPTH_COMPRESSION_SIMULATION = "raw";
const char * ROS_IMAGE1_TOPIC_REAL_WORLD = "/kinect_head/rgb/image_rect_color"; // "/kinect2/qhd/image_color_rect";
const char * ROS_IMAGE1_TOPIC_SIMULATION = "/head_mount_kinect/rgb/image_raw";
const string ROS_IMAGE1_COMPRESSION_REAL_WORLD = "compressed";
const string ROS_IMAGE1_COMPRESSION_SIMULATION = "compressed";
const char * ROS_POINTCLOUD_TOPIC_REAL_WORLD = "/kinect_head/depth/points"; // "/kinect_head/depth_registered/points"; // "/kinect2/qhd/points";
const char * ROS_POINTCLOUD_TOPIC_SIMULATION = "/head_mount_kinect/depth/points";
const string CAMERA1_FRAME_ID_REAL_WORLD = "head_mount_kinect_ir_optical_frame"; // "head_mount_kinect_rgb_optical_frame"
const string CAMERA1_FRAME_ID_SIMULATION = "head_mount_kinect_ir_optical_frame";

//Secondary camera settings
const bool ROS_IMAGE2_ENABLE = true;
const char * ROS_IMAGE2_TOPIC_REAL_WORLD = "/l_forearm_cam/image_rect_color"; //"/wide_stereo/left/image_color";
const char * ROS_IMAGE2_TOPIC_SIMULATION = "/l_forearm_cam/image_rect_color";
const string ROS_IMAGE2_COMPRESSION_REAL_WORLD = "compressed"; // "raw" // "theora
const string ROS_IMAGE2_COMPRESSION_SIMULATION = "compressed";
const string ROS_IMAGE2_CAMERA_INFO_TOPIC_REAL_WORLD = "/l_forearm_cam/camera_info"; //"/wide_stereo/left/camera_info";
const string ROS_IMAGE2_CAMERA_INFO_TOPIC_SIMULATION = "/l_forearm_cam/camera_info";
const string CAMERA2_FRAME_ID_REAL_WORLD = "l_forearm_cam_optical_frame"; //"wide_stereo_l_stereo_camera_optical_frame";
const string CAMERA2_FRAME_ID_SIMULATION = "l_forearm_cam_optical_frame";

//Outside view camera settings
const bool SHOW_OUTSIDE_VIEW_IN_SIMULATION = true;
const char * ROS_OUTSIDE_VIEW_TOPIC = "/camera/rgb/image_raw";
const string ROS_OUTSIDE_VIEW_COMPRESSION = "compressed";

//Active vision settings
const volatile activeVisionModes ACTIVE_VISION_MODE = activeVisionModes::Deterministic; //Options: None, Deterministic, Intelligent

//Deterministic triangulation planning settings
const string TRIANGULATION_PLANNING_BASE_FRAME = "l_shoulder_pan_link";
const string ROBOT_BASE_FRAME = "base_link";
const float ELBOW_FIXED_VALUE = -M_PI / 4.0;
const float UPPER_ARM_ROLL_VALUE = M_PI / 2.0;

//ROS publishing settings
const bool PUBLISH_TO_ROS = true;
const char * ROS_NODE_NAME = "CV_Node";
const char * ROS_SERVICE_ADVERTISE_ADDRESS = "/CV_Objects";

//3D world localization settings (in camera coordinate)
const bool ENABLE_3D_WORLD_LOCALIZATION = true;
const double ROS_FRAME_TRANSFORM_WAIT_TIME = 5.0; // measured in seconds

//OpenCV settings
const bool ENABLE_OPENCV_CUDA = true;

//Temporal display settings
const int WAIT_KEY_MILLISECOND = 1;

//Specifying the input image sizes
const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;

/*GUI display setting*/
const char * GUI_COMPONENTS_LOCATION = "./GUIComponents/";
const int GUI_HEIGHT = 980;
const int GUI_WIDTH = 1400;
//Specify the approximate size of the screen to display the output windows in the proper place
const int Screen_Height = 1080; // 800
const int Screen_Width = 1920; // 1400

//Output window names
const char * MAIN_OUTPUT = "Main Camera";
const char * SECONDARY_OUTPUT = "Secondary Camera";
const char * FUSED_OUTPUT = "Fused Output";
const char * AMALGAMATED_OUTPUT = "Amalgamated Output";
const bool AMALGAMATED_VIEW_ENABLED = true;

//Settings for saving snapshot images
const char * SNAPSHOTS_LOCATION = "./Snapshots/";
const int SANPSHOT_QUALITY = 95; //Default value for ".jpg" format is 95.

//Settings for recording videos
// "MJPG" or "PIM1" or "MPEG"
const string RECORDING_ENCODING = "MJPG";
const char * VIDEO_RECORDING_LOCATION = "./RecordedVideos/";
const double RECORDING_FPS = 20;

//Preprocessing settings
const bool PREPROCESSING_BLUR_TEST_IMAGES = false;
const bool PREPROCESSING_EQUALIZE_HISTOGRAM = false;

//Detector and classifier type
const classifierTypes CLASSIFIER_TYPE = classifierTypes::SVM;
const detectorTypes DETECTOR_TYPE = detectorTypes::Sliding_Window;

//Training settings
const bool DATA_AUGMENTATION = true;
const bool DATA_AUGMENTATION_BACKGROUND_CLASS = true;
const string TRAINING_SOURCE_FOLDER = "./TrainingData";
const string TRAINING_OBJECTS_LIST = "Objects_List.txt";
const string TRAINING_OBJECTS_EXTENSION = ".jpg";
const string BACKGROUND_CLASS_NAME = "Background";

//Classifier settings
const bool DIFFERENT_PROBABILITY_THRESHOLD_PER_CLASS = false;
const float GLOBAL_OBJECT_PROBABILITY_THRESHOLD = 0.7;
const vector<float> OBJECT_PROBABILITY_THRESHOLD_PER_CLASS = {0, 0.9, 0.85, 0.99, 0.7, 0.9, 0.9, 0.85, 0.9}; //Threshold probabilities for each class in order of their listing in the training name list file. Note:: The "Background" class probability is trivially included to prevent ordering problems.
const char * CLASSIFIER_SOURCE_FOLDER = "./LearningModels";

//Detector settings
const bool DISCARD_CLOSE_IDENTICAL_BOUNDING_BOXES = true;
const int BOUNDING_BOX_PROXIMITY_DISTANCE = 30;

//Object filtering
//Size filtering *** In the case of blob based detections, like intensity thresholding and background subtraction
const bool REJECT_ABNORMAL_OBJECT_SIZES = false;
const int MIN_OBJECT_SIZE = 300; //in pixels
const int MAX_OBJECT_SIZE = 5000; //in pixels
//Depth filtering
const bool REJECT_ABNORMAL_DISTANCES = false;
const int MIN_DEPTH = 17; //in depth image intensity value
const int MAX_DEPTH = 26; //in depth image intensity value
//Intensity filtering (Too dark areas). Based on the intensity thresholding settings *** Not useful for instensity-based detection, since it already does this kind of rejection
const bool REJECT_INTENSITY_BASED = true;
//Discard the boundary regions of the input images
const bool REJECT_INPUT_IMAGE_BOUNDARIES = false;
const float LENGTH_WIDTH_DISCARD_RATIO = 0.4;

//Intensity thresholding settings
const int INTENSITY_THRESHOLD = 60; //40

//Specifying the ratio of morphological structuring element's dimension to the image dimension
const double STRUCTURING_ELEMENT_TO_IMAGE_RATIO = 0.02;

//Mixture of Gaussians settings
const int MOG_VARIABLE_THRESHOLD = 16;
const int MOG_HISTORY = 10000;
const double MOG_LEARNING_RATE_NOMINATOR = 0.2; // denominator is MOG_HISTORY

//Sliding window settings
const Size SLIDING_WINDOW_SIZE = Size(64, 64);
const int SLIDING_WINDOW_STEP_SIZE = 10;
const int SLIDING_WINDOW_INITIAL_MAGNIFICATION = 1;
const int IMAGE_PYRAMID_NUMBER_OF_LAYERS = 2;
const int NUMBER_OF_WINDOW_SLIMMINGS_EACH_DIMENSION = 0; //For each dimension (i.e. vertical or horizontal), every time the size is cut into half
const int NON_MAXIMUM_SUPPRESSION_OVERLAP_THRESHOLD = 0.5;
const int NON_MAXIMUM_SUPPRESSION_OBJECT_NUMBER = 150;

//Matching
const MatchingTypes MATCHING_TYPE = MatchingTypes::ShortestDistance;
const double STANDARD_DEVIATION_RATIO = 1.5;
const int MAXIMUM_MATCHING_DISTANCE_DENOMINATOR = 10; // Maximum distance = min(image height, image width) / denominator

//Feature descriptors
//Color histogram
const int COLOR_HISTOGRAM_BIN_NUMBER = 50; //Max: 2 * 256^2
//HOG
const int REDUCED_HOG_NUMBER_OF_FEATURES = 60; //if feature reduction is enabled
const Size HOG_WINDOWS_SIZE = Size(64, 64);
const Size HOG_BLOCK_SIZE = Size(32, 32);
const Size HOG_BLOCK_STRIDE = Size(16, 16);
const Size HOG_CELL_SIZE = Size(16, 16);
const int HOG_BIN_NUMBER = 5;

//Feature reduction settings
const bool ENABLE_FEATURE_REDUCTION = true;
const string FEATURE_REDUCTION_SOURCE_FOLDER = "./LearningModels";
const string FEATURE_REDUCTION_METHOD = "PCA->LDA"; // "LDA" or "PCA"
const int PCA_TO_LDA_RATIO = 6; //if FEATURE_REDUCTION_METHOD is "PCA->LDA", this defines the fraction of PCA features after applying LDA

//Python classification settings
const char * PYTHON_TRAINING_MODULE_NAME = "VisionTraining_Python";
const char * PYTHON_CLASSIFIER_MODULE_NAME = "Classifier_Python";
const char * PYTHON_CLASSIFIER_LOADING_METHOD = "LoadLearningMemory";
const char * PYTHON_FEATURE_CLASSIFIER_TRAINING_CLASS = "Feature_Classifier_Training";
const char * PYTHON_FEATURE_CLASSIFIER_CLASS = "Feature_Classifier";

//Support Vector Machine settings
const char * SCIKIT_LEARN_SVM_KERNEL = "rbf"; // "rbf" "poly" "sigmoid" "linear"
const char * PYTHON_SVM_CLASSIFIER_METHOD = "SupportVectorMachine";

//Confidence reckoning
const CONFIDENCE_MEASURES CONFIDENCE_MEASURE = CONFIDENCE_MEASURES::firstMaxOverSecondMax;
const double CONFIDENCE_FIRST_MAX_TO_SECOND_MAX_RATIO = 100;
const double CONFIDENCE_UNKNOWN_PROBABILITY_THRESHOLD = 0.4;

//Tracker settings
const TRACKER_TYPES TRACKER_TYPE = TRACKER_TYPES::MedianFlow;
const int TRACKING_INTERVAL = 50; //The number of trackings of objects before another round of detection is executed
const int TRACKER_RETRY_AFTER_FAIL = 3;
const bool COMPUTE_TRACKING_CERTAINTY = true;

//Data fusion settings
const DECISION_FUSION_TYPES DECISION_FUSION_TYPE = DECISION_FUSION_TYPES::Bayesian; //DempsterShafer or Bayesian
const string DEMPSTER_SHAFER_ALL_OBJECTS_CATEGORY_NAME = "Unknown-Object";

//Additional output settings
const bool STORE_PROBABILITY_VECTORS = false;
const bool VERBOSE_PRINT_PROBABILITIES = false;
const bool DISPLAY_MID_PROCESS_FRAMES = false;
const bool DISPLAY_TRANSFORMED_CENTROID_IN_SECONDARY_VIEW = true;
const bool DISCARD_UNCERTAIN_FUSED_DETECTIONS = false;
const bool SHOW_UNMATCHED_SECONDARY_DETECTIONS = true;

//Detection wait (linger) settings (all units are seconds)
const int DETECTION_LINGER_TIME = 5;
const int HAND_LINGER_TIME_AFTER_DETECTION = 2;
const int MOVE_FORWARD_REST_TIME = 1;
const int RESPAWNING_LINGER_AFTER_HAND_MOVEMENT = 8;

//Collision avoidance settings
const float AVOIDANCE_AREA_CENTER_WIDTH = 0.65;
const float AVOIDANCE_AREA_LENGTH = 1.8;
const float AVOIDANCE_AREA_WIDTH = 0.95;
const float AVOIDANCE_AREA_HEIGHT = 0.75; // + 0.20
const float AVOIDANCE_AREA_TOLERANCE = 0.05;

//Object placement settings
const float PLACEMENT_AREA_LENGTH = 1.2;
const float PLACEMENT_AREA_WIDTH = 0.95;
const int PLACEMENT_AREA_DIVISIONS_PER_AXIS = 20;
const float PLACEMENT_AREA_UNIT_HEIGHT = 0.5;
const string OBJECT_DIMENSIONS_FILE_ADDRESS = "./SimulationObjectsInformation/ObjectsDimensions.txt";
const string ARTIFICIAL_OBJECTS_GENERIC_NAMES[3] = {"Box", "Pyramid", "Tube"};
const int ARTIFICIAL_OBJECTS_PER_GENERIC_LABEL[3] = {13, 14, 13};
const int STRAIGHT_PLACEMENT_COLUMN_GAP = 5;
const float OUTSIDE_CAMERA_POSE[3] = {2.67, 1.10, -2.67}; // x, y, yaw
const bool PRINT_PLACEMENT_AVAILABILITY_MAPS = false;
const bool PRINT_START_FINISH_MESSAGES = false;
