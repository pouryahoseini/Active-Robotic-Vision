/*
 * SimulationWorld.h
 *
 *  Created on: Mar 8, 2018
 *      Author: pourya
 */

//Guard
#ifndef SOURCECODE_SIMULATIONWORLD_H_
#define SOURCECODE_SIMULATIONWORLD_H_

//Headers
#include <unistd.h>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <regex>
#include <algorithm>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <atomic>

#include "GlobalDefinitions.h"

//Namespaces
using namespace std;

//Enumeration of Gazebo object types
enum simulationObjectType {realistic, artificial, both};

//Enumeration of Gazebo placement strategies
enum simulationPlacementStrategy {randomized, straight};

//Declaring dummy atmic variables to be used as default arguments for the below functions
atomic<bool> dummy_inProgress;
atomic<time_t> dummy_timeOfFinish;

//Global mutex to protect object spawning indicators
mutex spawningInProgressMutex;

//The class to deal with the Gazebo simulation world
class SimulationWorld
{
public:
  //Constructor
  SimulationWorld();

  //Destructor
  ~SimulationWorld();

  //Function to spawn a layout of objects on the table based on a predefined sequence of layouts
  void spawnNextLayout(runModes runMode = runModes::Test, atomic<bool> & inProgress = dummy_inProgress, atomic<time_t> & timeOfFinish = dummy_timeOfFinish, int waitBeforeSpawn = 0);

  //Function to perform the preliminary tasks
  void runInitialTasks();

private:
  /* Interface function to run an epoch of object placement on the table */
  //numberOfObjects: 0 means lay objects until the table is full or objects are run out. Any other positive number is the maximum number of objects placed before they are run out or the number of lines retriction is satisfied.
  //objectType: artificial, realistic, both
  //placementStrategy: random, straight. Straight: starts from the front of a column and goes backward, then if necessary to fill more objects restarts from a right column
  //straightPlacement_startingColumn: Only effective when "straight" placement strategy is chosen. A number not in the bound (ie. negative or equal or more than PLACEMENT_AREA_DIVISIONS_PER_AXIS) means random selection of the starting column.
  //straightPlacement_maxColumns: Only effective when "straight" placement strategy is chosen. Sets the maximum number of columns. It overrides the termination condition set by "numberOfObjects" if the number of objects are more than the number of columns. A number not in the bound (ie. zero or negative or more than PLACEMENT_AREA_DIVISIONS_PER_AXIS) means no limitation on the number of columns.
  void layObjects(int numberOfObjects = 0, enum simulationObjectType objectType = simulationObjectType::realistic, enum simulationPlacementStrategy placementStrategy = simulationPlacementStrategy::randomized, int straightPlacement_startingColumn = -1, int straightPlacement_maxColumns = -1);

  //Struct to hold object dimensions info of an object
  struct objectDimensionsInfo
  {
    float width, length, height;
    int index;
    string name;
  };

  //Struct to hold name and index of an object
  struct objectID
  {
    int index;
    string name;
  };

  //Struct to hold placement feasibility info of a cell on the table
  struct cellPlacementFeasibilityInfo
  {
    bool occupied, feasible;
  };

  //Struct to hold location of a cell on the table
  struct cellLocation
  {
    int row, column;
  };

  //Struct to hold a square. Each entry cell number is included in the square
  struct squareMap
  {
    int left, right, top, bottom;
  };

  //Struct to hold location of an object
  struct location
  {
    float x, y;
  };

  //Enumeration of pre-defined yaws of objects
  enum yaw {turn0, turn90, turn180, turn270};

  //Function to clear the table from objects
  void clearTable();

  //Function to choose a random object from the remaining object that are not spawned yet. It returns true if there was any object remaining to choose
  bool chooseRandomObject(enum simulationObjectType input_objectType, int & output_objectIndexNumber, enum yaw & output_objectYaw);

  //Function to place an object behind another one. It returns true if it was able to place the object
  bool placeBehindObject(int objectIndexNumber, int referenceObjectIndexNumber, enum yaw objectYaw);

  //Function to place an object on a random location. It returns true if it was successful
  bool placeRandomly(int objectIndexNumber, enum yaw objectYaw);

  //Function to place an object in front of a column on the table
  bool placeColumnFront(int objectIndexNumber, int column, enum yaw objectYaw);

  //A general function to place an object given a correct and feasible placement location
  void placeObject(int objectIndexNumber, enum yaw objectYaw, int input_row, int input_column);

  //Function to choose a random location for a given object. It returns true if it was able to place the object
  bool chooseRandomLocation(int objectIndexNumber, enum yaw objectYaw, int & output_row, int & output_column);

  //Function to get the feasible placement areas for a given object
  void getFeasiblePlacementAreas(int objectIndexNumber, enum yaw objectYaw);

  //Function to update the occupied areas on the table
  void updateOcciupiedAreas(squareMap newObjectMap);

  //Function to clear feasibility information of cells on the table to place an object
  void clearFeasibilityFlags();

  //Function to get object width and length based on the chosen yaw
  void getDimensionsAfterRotation(int objectIndexNumber, enum yaw objectYaw, int & outputLength, int & outputWidth);

  //Function to read the dimensions of the objects in the Gazebo library
  void readObjectDimensions();

  //Function to spawn an object
  void spawnObject(string objectName, float x, float y, float yaw, float z);

  //Function to delete an object
  void deleteObject(string objectName);

  //The function to create a subprocess for spawning and deleting Gazebo models and terminate that if Gazebo is not responsive withing a time frame
  void callGazebo(const string command);

  //Function to move an object
  void moveObject(string objectName, float x, float y, float yaw, float z);

  //Function to move an object back to its initial location
  void moveBack(objectID object_ID);

  //Function to spawn objects in a distant place from the robot
  void initialSpawning();

  stringstream artificialObjectName, command;
  const float ground_z = 0, table_z = 0.74; // The height of the table
  float center_x, center_y, maxLength, maxWidth, moveBack_x, moveBack_y;
  char tableObjectName[6] = "table", cameraName[15] = "Camera_Outside";
  ifstream objectDimensionsFile;
  vector<objectID> spawnedObjects;
  objectID currentObjectID;
  string line, moveBack_objectName;
  struct objectDimensionsInfo currentObjectDimensionStruct;
  vector<struct objectDimensionsInfo> objectDimensionsArray, objectDimensionsArray_shuffled, * shuffledArray, artificialObjectDimensionsArray, realisticObjectDimensionsArray, artificialObjectDimensionsArray_shuffled, realisticObjectDimensionsArray_shuffled;
  struct cellPlacementFeasibilityInfo ** cell;
  int randomIndex, row, column, requestedColumn, length, width, center_row, center_column, currentIndex, selected_objectIndexNumber, columnNumber, previousObjectIndex;
  int addedToMax, roundCounter = 0, cellCounter, threadCounter, layoutCounter = 0;
  vector<cellLocation> feasibleCells;
  cellLocation currentCellLocation, * spawnedObjectCenterLocations;
  squareMap currentSquareMap;
  yaw selected_yaw;
  bool newColumn, failedPlacement;
  vector<thread> spawnThread, moveThread, moveBackThread;
  mutex gazeboMutex;
  vector<location> initialObjectLocations;
  location currentLocation;
  mutex publicFunctions_Lock;
};

#endif /* SOURCECODE_SIMULATIONWORLD_H_ */
