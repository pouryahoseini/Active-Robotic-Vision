/*
 * SimulationWorld.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: pourya
 */

//Header
#include "SimulationWorld.h"

/*******************************/
//Function to initialize the simulation world
SimulationWorld::SimulationWorld()
{
  //Do operations only if simulation is enabled
  if(SIMULATION_WORLD_ENABLE == true)
    {
      //Construct the cell dynamic array
      cell = new cellPlacementFeasibilityInfo * [PLACEMENT_AREA_DIVISIONS_PER_AXIS];
      for(int i = 0; i < PLACEMENT_AREA_DIVISIONS_PER_AXIS; i++)
	cell[i] = new cellPlacementFeasibilityInfo[PLACEMENT_AREA_DIVISIONS_PER_AXIS];

      //Set random number generator seed
      srand(time(NULL));

      //Read object dimensions from file
      readObjectDimensions();
    }
}

/*******************************/
//Destructor
SimulationWorld::~SimulationWorld()
{
  //Clear table
  clearTable();

  //Do operations only if simulation is enabled
  if(SIMULATION_WORLD_ENABLE == true)
    {
      //Delete the cell dynamic array
      for(int i = 0; i < PLACEMENT_AREA_DIVISIONS_PER_AXIS; i++)
	delete[] cell[i];
      delete[] cell;

      //Delete the cell locations dynamic array
      delete [] spawnedObjectCenterLocations;
    }
}


/*******************************/
//Function to perform the preliminary tasks
void SimulationWorld::runInitialTasks()
{
  //Spawn a table
  spawnObject(tableObjectName, AVOIDANCE_AREA_CENTER_WIDTH, 0, M_PI/2, ground_z);

  //Spawn a camera for outside view of the robot in action
  spawnObject(cameraName, OUTSIDE_CAMERA_POSE[0], OUTSIDE_CAMERA_POSE[1], OUTSIDE_CAMERA_POSE[2], ground_z);

  //Initially spawn objects in a distant place
  initialSpawning();

  return;
}

/*******************************/
//Function to spawn a layout of objects on the table based on a predefined sequence of layouts
void SimulationWorld::spawnNextLayout(runModes runMode, atomic<bool> & inProgress, atomic<time_t> & timeOfFinish, int waitBeforeSpawn)
{cout<<endl<<"Spawning started"<<endl;
  //Lock the public functions mutex
  publicFunctions_Lock.lock();

  //Signal the placement is in progress
  spawningInProgressMutex.lock();
  inProgress = true;

  //Sleep for the specified time frame
  sleep(waitBeforeSpawn);

  //Decide on the testing or locomotion training patterns
  if(runMode == runModes::Test)
    {
      switch((layoutCounter %= 4))
      {
	case 0:
	  layObjects(1, simulationObjectType::realistic, simulationPlacementStrategy::randomized, -1, -1);
	  break;
	case 1:
	  layObjects(-1, simulationObjectType::artificial, simulationPlacementStrategy::straight, -1, -1);
	  break;
	case 2:
	  layObjects(-1, simulationObjectType::realistic, simulationPlacementStrategy::straight, -1, -1);
	  break;
	case 3:
	  layObjects(-1, simulationObjectType::artificial, simulationPlacementStrategy::randomized, -1, -1);
	  break;
      }
    }
  else //runMode == runModes::Training_Locomotion
    {
      switch((layoutCounter %= 4))
      {
	case 0:
	  layObjects(-1, simulationObjectType::realistic, simulationPlacementStrategy::randomized, 3, -1);
	  break;
	case 1:
	  layObjects(-1, simulationObjectType::artificial, simulationPlacementStrategy::straight, 3, -1);
	  break;
	case 2:
	  layObjects(-1, simulationObjectType::realistic, simulationPlacementStrategy::straight, 3, -1);
	  break;
	case 3:
	  layObjects(-1, simulationObjectType::artificial, simulationPlacementStrategy::randomized, 3, -1);
	  break;
      }
    }

  //Increase the layout counter by one
  //layoutCounter++;

  //Signal the placement is done
  inProgress = false;
  spawningInProgressMutex.unlock();

  //Save the time of finishing the spawning task
  timeOfFinish = time(NULL);

  //Unlock the public functions mutex
  publicFunctions_Lock.unlock();
cout<<endl<<"Spawn done"<<endl;
  return;
}

/*******************************/
/* Interface function to run an epoch of object placement on the table */
//numberOfObjects: Zero or a negative number means lay objects until the table is full or objects are run out. Any other positive number is the maximum number of objects placed before they are run out or the number of lines retriction is satisfied.
//objectType: artificial, realistic, both
//placementStrategy: random, straight. Straight: starts from the front of a column and goes backward, then if necessary to fill more objects restarts from a right side column
//straightPlacement_startingColumn: Only effective when "straight" placement strategy is chosen. A number not in the bound (ie. negative or equal or more than PLACEMENT_AREA_DIVISIONS_PER_AXIS) means random selection of the starting column.
//straightPlacement_maxColumns: Only effective when "straight" placement strategy is chosen. Sets the maximum number of columns. It overrides the termination condition set by "numberOfObjects" if the number of objects are more than the number of columns. A number not in the bound (ie. zero or negative or more than PLACEMENT_AREA_DIVISIONS_PER_AXIS) means no limitation on the number of columns.
void SimulationWorld::layObjects(int numberOfObjects, enum simulationObjectType objectType, enum simulationPlacementStrategy placementStrategy, int straightPlacement_startingColumn, int straightPlacement_maxColumns)
{
  //Clear table
  clearTable();

  //If there is no limit on the number of objects to be laid (ie. numberOfObjects == 0), set it to the maximum possible placement locations (ie. number of cells) plus 1
  if(numberOfObjects <= 0)
    numberOfObjects = (PLACEMENT_AREA_DIVISIONS_PER_AXIS * PLACEMENT_AREA_DIVISIONS_PER_AXIS) + 1;

  //Check the placement strategy
  if(placementStrategy == simulationPlacementStrategy::randomized)
    {
      //Choose a random object and continue while the termination condition is not met (numberOfObjects > 0)
      while((numberOfObjects > 0) && (chooseRandomObject(objectType, selected_objectIndexNumber, selected_yaw)))
	{
	  //Decrement the number of objects to be laid
	  numberOfObjects--;

	  //Place the randomly selected object in a random location. If not successful, do not try other placements
	  if(! placeRandomly(selected_objectIndexNumber, selected_yaw))
	    break;
	}
    }
  else if(placementStrategy == simulationPlacementStrategy::straight)
    {
      //Check if the maximum column number is out of the bound, set it to unlimited (maximum possible plus one)
      if((straightPlacement_maxColumns <= 0) || (straightPlacement_maxColumns > PLACEMENT_AREA_DIVISIONS_PER_AXIS))
	straightPlacement_maxColumns = PLACEMENT_AREA_DIVISIONS_PER_AXIS;

      //Decide on the starting column randomly if it set to a number out of the bounds
      if((straightPlacement_startingColumn < 0) || (straightPlacement_startingColumn >= PLACEMENT_AREA_DIVISIONS_PER_AXIS))
	straightPlacement_startingColumn = rand() % PLACEMENT_AREA_DIVISIONS_PER_AXIS;

      //Set the column number to one
      columnNumber = straightPlacement_startingColumn;

      //Set the new line boolean variable to true
      newColumn = true;

      //Set the boolean variable for a failed placement to false
      failedPlacement = false;

      //Set the added value to the max columns to zero
      addedToMax = 0;

      //Choose a random object and continue while the termination condition is not met (numberOfObjects > 0)
      while((numberOfObjects > 0) && ((((columnNumber - straightPlacement_startingColumn) / STRAIGHT_PLACEMENT_COLUMN_GAP) + 1) <= (straightPlacement_maxColumns + (addedToMax / STRAIGHT_PLACEMENT_COLUMN_GAP))) && (columnNumber < PLACEMENT_AREA_DIVISIONS_PER_AXIS) && (failedPlacement ? true : (chooseRandomObject(objectType, selected_objectIndexNumber, selected_yaw))))
	{
	  //Check if it a new column placement
	  if(newColumn == true) //New column
	    {
	      //Place the object in front of the column
	      if(placeColumnFront(selected_objectIndexNumber, columnNumber, selected_yaw))
		{
		  //Decrement the number of objects to be laid
		  numberOfObjects--;

		  //Set the new line boolean variable to false, so that other objects are placed behind each other in the same column
		  newColumn = false;

		  //Set the boolean variable for a failed placement to false
		  failedPlacement = false;

		  //Save the object index
		  previousObjectIndex = selected_objectIndexNumber;
		}
	      else
		{
		  //Increment the column number
		  columnNumber++;

		  //Set the boolean variable for a failed placement to true
		  failedPlacement = true;

		  //Increment the added value to the max columns
		  addedToMax++;
		}
	    }
	  else //Not a new column
	    {
	      //Place the object behind an object in the current column
	      if(placeBehindObject(selected_objectIndexNumber, previousObjectIndex, selected_yaw)) //Placement successful
		{
		  //Decrement the number of objects to be laid
		  numberOfObjects--;

		  //Set the boolean variable for a failed placement to false
		  failedPlacement = false;

		  //Save the object index
		  previousObjectIndex = selected_objectIndexNumber;
		}
	      else //Placement unsuccessful
		{
		  //Increment the column number
		  columnNumber += STRAIGHT_PLACEMENT_COLUMN_GAP;

		  //Set the new line boolean variable to true, so that the next object is placed in the column
		  newColumn = true;

		  //Set the boolean variable for a failed placement to true
		  failedPlacement = true;
		}
	    }
	}
    }
  else
    {
      cerr << "\nin function SimulationWorld::layObjects: The placement strategy is not recognized.\n" << endl;
      exit(EXIT_FAILURE);
    }

  //Join the spawning threads
  for(threadCounter = 0; threadCounter < moveThread.size(); threadCounter++)
    moveThread[threadCounter].join();

  //Clear the vector of threads
  moveThread.clear();

  //Print the operation conclusion message with the object placement round number
  if(PRINT_START_FINISH_MESSAGES)
    cout << endl << "\nSimulator: Finished placement round " << (++roundCounter) << endl;

  return;
}

/*******************************/
//Function to place an object behind another one. It returns true if it was able to place the object
bool SimulationWorld::placeBehindObject(int objectIndexNumber, int referenceObjectIndexNumber, enum yaw objectYaw)
{
  //Set the object column
  requestedColumn = spawnedObjectCenterLocations[referenceObjectIndexNumber].column;

  //Check the location of the reference object to be in the bounds
  if((spawnedObjectCenterLocations[referenceObjectIndexNumber].row < 0) || (spawnedObjectCenterLocations[referenceObjectIndexNumber].row >= PLACEMENT_AREA_DIVISIONS_PER_AXIS) || (requestedColumn < 0) || (requestedColumn >= PLACEMENT_AREA_DIVISIONS_PER_AXIS))
    {
      printf("\nplaceBehindObject: Reference object's placement position is not in the bounds.\n\n");
      return false;
    }

  //Return false if the reference object is not placed yet
  if(! cell[spawnedObjectCenterLocations[referenceObjectIndexNumber].row][requestedColumn].occupied)
    {
      cout << "\nplaceBehindObject: Reference object is not placed on the table yet.\n" << endl;
      return false;
    }

  //Get feasible placement areas
  getFeasiblePlacementAreas(objectIndexNumber, objectYaw);

  //Try the locations behind the reference object, starting from the closest position
  for(row = spawnedObjectCenterLocations[referenceObjectIndexNumber].row - 1; row >= 0; row --)
    if(cell[row][requestedColumn].feasible && (! cell[row][requestedColumn].occupied))
      {
	//Place the object
	placeObject(objectIndexNumber, objectYaw, row, requestedColumn);

	//Reset feasibility of cells
	clearFeasibilityFlags();

	return true;
      }

  //Reset feasibility of cells
  clearFeasibilityFlags();

  return false;
}

/*******************************/
//Function to place an object on a random location. It returns true if it was successful
bool SimulationWorld::placeRandomly(int objectIndexNumber, enum yaw objectYaw)
{
  //Get a random location. If it is not possible, return false
  if(! chooseRandomLocation(objectIndexNumber, objectYaw, center_row, center_column))
    return false;

  //Place the object
  placeObject(objectIndexNumber, objectYaw, center_row, center_column);

  return true;
}

/*******************************/
//Function to place an object in front of a column on the table
bool SimulationWorld::placeColumnFront(int objectIndexNumber, int column, enum yaw objectYaw)
{
  //Check the column to be in the bounds
  if((column < 0) || (column >= PLACEMENT_AREA_DIVISIONS_PER_AXIS))
    {
      printf("\nSimulationWorld::placeColumnFront: The specified placement column is not in the bounds.\n\n");
      return false;
    }

  //Get feasible placement areas
  getFeasiblePlacementAreas(objectIndexNumber, objectYaw);

  //Try the locations in the specified column, starting from the closest position to the robot (column front)
  for(row = PLACEMENT_AREA_DIVISIONS_PER_AXIS - 1; row >= 0; row --)
    if(cell[row][column].feasible && (! cell[row][column].occupied))
      {
	//Place the object
  	placeObject(objectIndexNumber, objectYaw, row, column);

  	//Reset feasibility of cells
  	clearFeasibilityFlags();

  	return true;
      }

  //Reset feasibility of cells
  clearFeasibilityFlags();

  return false;
}

/*******************************/
//Function to choose a random object from the remaining object that are not spawned yet. It returns true if there was any object remaining to choose
bool SimulationWorld::chooseRandomObject(enum simulationObjectType input_objectType, int & output_objectIndexNumber, enum yaw & output_objectYaw)
{
  //Decide on the type of objects being selected
  if(input_objectType == simulationObjectType::artificial)
    shuffledArray = & artificialObjectDimensionsArray_shuffled;
  else if (input_objectType == simulationObjectType::realistic)
    shuffledArray = & realisticObjectDimensionsArray_shuffled;
  else
    shuffledArray = & objectDimensionsArray_shuffled;

  //Return false if no objects are left in the vector of object dimensions
  if((* shuffledArray).empty())
    return false;

  //Pop out an object model from the shuffled vector of object dimensions
  currentObjectDimensionStruct = (* shuffledArray).back();
  (* shuffledArray).pop_back();
  output_objectIndexNumber = currentObjectDimensionStruct.index;

  //Randomly choose a yaw from the four options: 0, Pi/2, Pi, and 3*Pi/2 degrees
  output_objectYaw = (yaw) (rand() % 4);

  return true;
}

/*******************************/
//Function to choose a random location for a given object. It returns true if it was able to place the object
bool SimulationWorld::chooseRandomLocation(int objectIndexNumber, enum yaw objectYaw, int & output_row, int & output_column)
{
  //Get feasible placement areas
  getFeasiblePlacementAreas(objectIndexNumber, objectYaw);

  //Extract all the feasible locations and save them in a variable array
  for(row = 0; row < PLACEMENT_AREA_DIVISIONS_PER_AXIS; row++)
    for(column = 0; column < PLACEMENT_AREA_DIVISIONS_PER_AXIS; column++)
      if((cell[row][column].feasible == true) && (cell[row][column].occupied == false))
	{
	  currentCellLocation.row = row;
	  currentCellLocation.column = column;
	  feasibleCells.push_back(currentCellLocation);
	}

  //If no feasible cells are found return false
  if(feasibleCells.empty())
    return false;

  //Select one of the feasible cells randomly and compute its centeral location
  randomIndex = rand() % feasibleCells.size();
  output_row = feasibleCells[randomIndex].row;
  output_column = feasibleCells[randomIndex].column;

  //Reset feasibility of cells
  clearFeasibilityFlags();

  return true;
}

/*******************************/
//A general function to place an object given a correct and feasible placement location
void SimulationWorld::placeObject(int objectIndexNumber, enum yaw objectYaw, int input_centerRow, int input_centerColumn)
{
  //Compute the centr location of the centeral placement cell
  center_x = -((input_centerRow + 0.5) * (PLACEMENT_AREA_WIDTH / PLACEMENT_AREA_DIVISIONS_PER_AXIS)) + AVOIDANCE_AREA_CENTER_WIDTH + (PLACEMENT_AREA_WIDTH / 2.0);
  center_y = -((input_centerColumn + 0.5) * (PLACEMENT_AREA_LENGTH / PLACEMENT_AREA_DIVISIONS_PER_AXIS)) + (PLACEMENT_AREA_LENGTH / 2.0);

  //Mark the placement location as occupied
  getDimensionsAfterRotation(objectIndexNumber, objectYaw, length, width);
  currentSquareMap.left = input_centerColumn - (length / 2);
  currentSquareMap.right = input_centerColumn + (length / 2);
  currentSquareMap.top = input_centerRow - (width / 2);
  currentSquareMap.bottom = input_centerRow + (width / 2);
  updateOcciupiedAreas(currentSquareMap);

  //Add the object name and index to the vector of spawned objects
  currentObjectID.name = objectDimensionsArray[objectIndexNumber].name;
  currentObjectID.index = objectDimensionsArray[objectIndexNumber].index;
  spawnedObjects.push_back(currentObjectID);

  //Save where the object is placed
  spawnedObjectCenterLocations[objectIndexNumber].row = input_centerRow;
  spawnedObjectCenterLocations[objectIndexNumber].column = input_centerColumn;

  //Spawn the object
  moveObject(objectDimensionsArray[objectIndexNumber].name, center_x, center_y, (M_PI / 2.0) * objectYaw, table_z);

  //Print a message if printing availability maps is enabled
  if(PRINT_PLACEMENT_AVAILABILITY_MAPS)
    cout << endl << "-----Object Placed in (" << input_centerRow << ", " << input_centerColumn << ")" << endl;

  return;
}

/*******************************/
//Function to get the feasible placement areas for a given object
void SimulationWorld::getFeasiblePlacementAreas(int objectIndexNumber, enum yaw objectYaw)
{
  //Set the length and width of the object based on the chosen yaw
  getDimensionsAfterRotation(objectIndexNumber, objectYaw, length, width);

  //Mark feasible areas by doing a morphological dilation of the occupied cells with a square structuring element equal in size to the object
  //Search through all the cells in the table mesh. It is an O(n) task (while n is number of the cells defined on the table).
  for(row = -1; row <= PLACEMENT_AREA_DIVISIONS_PER_AXIS; row++)
    for(column = -1; column <= PLACEMENT_AREA_DIVISIONS_PER_AXIS; column++)
      //If a cell is occupied mark all the cells' feasibility to false in a neighborhood equal to the size of the object
      if((column == -1) || (column == PLACEMENT_AREA_DIVISIONS_PER_AXIS) || (row == -1) || (row == PLACEMENT_AREA_DIVISIONS_PER_AXIS) || cell[row][column].occupied)
	for(int i = -(width / 2); i <= (width / 2); i++)
	  for(int j = -(length / 2); j <= (length / 2); j++)
	    {
	      //Prevent segmentation fault by keeping indexes in the bound
	      if(((row + i) < 0) || ((row + i) >= PLACEMENT_AREA_DIVISIONS_PER_AXIS) || ((column + j) < 0) || ((column + j) >= PLACEMENT_AREA_DIVISIONS_PER_AXIS))
		continue;

	      //Set the feasibility to false
	      cell[row + i][column + j].feasible = false;
	    }

  //Print the feasibility and occupancy maps if printing availability maps is enabled
  if(PRINT_PLACEMENT_AVAILABILITY_MAPS)
    {
      //Print the object's  name
      cout << "\n\n*****\nObjectName:  " << objectDimensionsArray[objectIndexNumber].name << endl;

      //Print titles
      cout << "\n\nFeasibility Map" << string(((2 * PLACEMENT_AREA_DIVISIONS_PER_AXIS) - 1 - 15 + 21), ' ') << "Occupancy Map\n";

      for(row = 0; row < PLACEMENT_AREA_DIVISIONS_PER_AXIS; row++)
	{
	  cout << endl;

	  //Print feasibility map
	  for(column = 0; column < PLACEMENT_AREA_DIVISIONS_PER_AXIS; column++)
	    cout << cell[row][column].feasible << " ";

	  cout << string(20, ' ');

	  //Print occupancy map
	  for(column = 0; column < PLACEMENT_AREA_DIVISIONS_PER_AXIS; column++)
	    cout << cell[row][column].occupied << " ";
	}
    }

  return;
}

/*******************************/
//Function to update the occupied areas on the table
void SimulationWorld::updateOcciupiedAreas(squareMap newObjectMap)
{
  //Check if the square is in the limits. If not impose the limits.
  if(newObjectMap.top < 0)
    newObjectMap.top = 0;

  if(newObjectMap.bottom > PLACEMENT_AREA_DIVISIONS_PER_AXIS - 1)
    newObjectMap.bottom = PLACEMENT_AREA_DIVISIONS_PER_AXIS - 1;

  if(newObjectMap.left < 0)
    newObjectMap.left = 0;

  if(newObjectMap.right > PLACEMENT_AREA_DIVISIONS_PER_AXIS - 1)
    newObjectMap.right = PLACEMENT_AREA_DIVISIONS_PER_AXIS - 1;

  //Mark all the cells of the square as occupied
  for(int i = newObjectMap.top; i <= newObjectMap.bottom; i++)
    for(int j = newObjectMap.left; j <= newObjectMap.right; j++)
      cell[i][j].occupied = true;

  return;
}

/*******************************/
//Function to clear the table from objects
void SimulationWorld::clearTable()
{
  //Delete all the spawned objects
  for(int i = 0; i < spawnedObjects.size(); i++)
    moveBack(spawnedObjects[i]);

  //Clear the variable array containing feasible cells for placement of an object
  feasibleCells.clear();

  //Clear the vector of spawned objects
  spawnedObjects.clear();

  //Clear the saved locations of objects
  for(cellCounter = 0; cellCounter < PLACEMENT_AREA_DIVISIONS_PER_AXIS; cellCounter++)
    {
      spawnedObjectCenterLocations[cellCounter].row = -1;
      spawnedObjectCenterLocations[cellCounter].column = -1;
    }

  //Refill the vector of object dimensions with another round of copying of then shuffling
  objectDimensionsArray_shuffled = objectDimensionsArray;
  random_shuffle(objectDimensionsArray_shuffled.begin(), objectDimensionsArray_shuffled.end());

  artificialObjectDimensionsArray_shuffled = artificialObjectDimensionsArray;
  random_shuffle(artificialObjectDimensionsArray_shuffled.begin(), artificialObjectDimensionsArray_shuffled.end());

  realisticObjectDimensionsArray_shuffled = realisticObjectDimensionsArray;
  random_shuffle(realisticObjectDimensionsArray_shuffled.begin(), realisticObjectDimensionsArray_shuffled.end());

  //Reinitialize the table cells
  for(int i = 0; i < PLACEMENT_AREA_DIVISIONS_PER_AXIS; i++)
    for(int j = 0; j < PLACEMENT_AREA_DIVISIONS_PER_AXIS; j++)
      {
	cell[i][j].occupied = false;
	cell[i][j].feasible = true;
      }

  //Join the deletion threads
  for(threadCounter = 0; threadCounter < moveBackThread.size(); threadCounter++)
    moveBackThread[threadCounter].join();

  //Clear the vector of deletion threads
  moveBackThread.clear();

  //Print the operation conclusion message with the object placement round number
  if(PRINT_START_FINISH_MESSAGES)
    cout << "Simulator: Finished cleaning the table of round " << roundCounter << endl;

  return;
}

/*******************************/
//Function to clear feasibility information of cells on the table to place an object
void SimulationWorld::clearFeasibilityFlags()
{
  //Clear the variable array containing feasible cells for placement of an object
  feasibleCells.clear();

  //Change all the cells to be feasible for object placement
  for(row = 0; row < PLACEMENT_AREA_DIVISIONS_PER_AXIS; row++)
    for(column = 0; column < PLACEMENT_AREA_DIVISIONS_PER_AXIS; column++)
      cell[row][column].feasible = true;

  return;
}

/*******************************/
//Function to get object width and length based on the chosen yaw
void SimulationWorld::getDimensionsAfterRotation(int objectIndexNumber, enum yaw objectYaw, int & outputLength, int & outputWidth)
{
  if((objectYaw == yaw::turn0) || (objectYaw == yaw::turn180))
    {
      outputLength = static_cast<int>(ceil(objectDimensionsArray[objectIndexNumber].length / (PLACEMENT_AREA_LENGTH / PLACEMENT_AREA_DIVISIONS_PER_AXIS)));
      outputWidth = static_cast<int>(ceil(objectDimensionsArray[objectIndexNumber].width / (PLACEMENT_AREA_WIDTH / PLACEMENT_AREA_DIVISIONS_PER_AXIS)));
    }
  else
    {
      outputLength = static_cast<int>(ceil(objectDimensionsArray[objectIndexNumber].width / (PLACEMENT_AREA_LENGTH / PLACEMENT_AREA_DIVISIONS_PER_AXIS)));
      outputWidth = static_cast<int>(ceil(objectDimensionsArray[objectIndexNumber].length / (PLACEMENT_AREA_WIDTH / PLACEMENT_AREA_DIVISIONS_PER_AXIS)));
    }

  return;
}

/*******************************/
//Function to read the dimensions of the objects in the Gazebo library
void SimulationWorld::readObjectDimensions()
{
  //Read the object dimensions file
  objectDimensionsFile.open(OBJECT_DIMENSIONS_FILE_ADDRESS.c_str(), ios::in);
  if(! objectDimensionsFile.is_open())
    {
      cerr << "\nThe object dimensions file cannot be opened.\n" << endl;
      exit(EXIT_FAILURE);
    }

  //get a line from the file
  getline(objectDimensionsFile, line);

  //Set the current object index number to zero
  currentIndex = 0;

  //Set the maximum length and width to zero
  maxLength = 0;
  maxWidth = 0;

  //Set the delimiter regular expression (regex)
  regex commaDelimiter(",");
  //char * line_CStyle, * word; /** For C-style parsing **/
  //char const * commaDelimiter_C = ","; /** For C-style parsing **/

  //Read all the file
  while(! objectDimensionsFile.eof())
    {
      //Remove white-space from the string
      for(int i = 0; i < line.length(); i++)
        if(line[i] == ' ') line.erase(i,1);

      /*** C++-style parsing ***/
      //Starting to possibly split the string into its objects
      regex_token_iterator<string::iterator> currentIterator(line.begin(), line.end(), commaDelimiter, -1); // , endIterator;

      //while (currentIterator != endIterator)
      //{
      //Get the object name
      currentObjectDimensionStruct.name = currentIterator->str();
      currentIterator++;

      //Get the dimensions
      currentObjectDimensionStruct.width = stof(currentIterator->str());
      currentIterator++;
      currentObjectDimensionStruct.length = stof(currentIterator->str());
      currentIterator++;
      currentObjectDimensionStruct.height = stof(currentIterator->str());
      currentIterator++;
      //}

      /*** C-style parsing  ***/
      /*
      //Copy the string on a C string
      line_CStyle = strdup(line.c_str());

      //Get the object name
      word = strtok(line_CStyle, commaDelimiter_C);
      currentObjectDimensionStruct.name = word;

      //Get the dimensions
      word = strtok(NULL, commaDelimiter_C);
      currentObjectDimensionStruct.width = static_cast<int>(ceil(atof(word) / (PLACEMENT_AREA_WIDTH / PLACEMENT_AREA_DIVISIONS_PER_AXIS)));
      word = strtok(NULL, commaDelimiter_C);
      currentObjectDimensionStruct.length = static_cast<int>(ceil(atof(word) / (PLACEMENT_AREA_LENGTH / PLACEMENT_AREA_DIVISIONS_PER_AXIS)));
      word = strtok(NULL, commaDelimiter_C);
      currentObjectDimensionStruct.height = static_cast<int>(ceil(atof(word) / PLACEMENT_AREA_UNIT_HEIGHT));
      */

      //Update the maximum length and width if necessary
      if(currentObjectDimensionStruct.width > maxWidth)
	maxWidth = currentObjectDimensionStruct.width;

      if(currentObjectDimensionStruct.length > maxLength)
	maxLength = currentObjectDimensionStruct.length;

      /* Push back the object dimensions struct in the vector */
      //Check if the current object name is one of the generic names for the artificial objects
      for(int i = 0; i < (sizeof(ARTIFICIAL_OBJECTS_PER_GENERIC_LABEL) / sizeof(ARTIFICIAL_OBJECTS_PER_GENERIC_LABEL[0])); i++)
	if(currentObjectDimensionStruct.name == ARTIFICIAL_OBJECTS_GENERIC_NAMES[i])
	  {
	    //Create a list of artificial objects and push back them into the array of artificial objects and the general array
	    for(int counter = 0; counter < ARTIFICIAL_OBJECTS_PER_GENERIC_LABEL[i]; counter++)
	      {
		//Set the index number
		currentObjectDimensionStruct.index = currentIndex;

		//Set the name of the artificial object
		artificialObjectName.str("");
		artificialObjectName << ARTIFICIAL_OBJECTS_GENERIC_NAMES[i] << "_" << counter;
		currentObjectDimensionStruct.name = artificialObjectName.str();

		//Push back object dimensions information
		artificialObjectDimensionsArray.push_back(currentObjectDimensionStruct); //Array of artificial objects
		objectDimensionsArray.push_back(currentObjectDimensionStruct); //Array of all objects

		//Increment index number
		currentIndex++;
		}
	    break;
	  }
	else if(i == (sizeof(ARTIFICIAL_OBJECTS_PER_GENERIC_LABEL) / sizeof(ARTIFICIAL_OBJECTS_PER_GENERIC_LABEL[0])) - 1) //If the object is not an artificial one and it is the last word to check
	  {
	    //Set the index number
	    currentObjectDimensionStruct.index = currentIndex;

	    //Push back object dimensions information
	    realisticObjectDimensionsArray.push_back(currentObjectDimensionStruct); //Array of realistic objects
	    objectDimensionsArray.push_back(currentObjectDimensionStruct); //Array of all objects

	    //Increment index number
	    currentIndex++;
	  }

      //get a line from the file
      getline(objectDimensionsFile, line);
    }

  //Close the file
  objectDimensionsFile.close();

  //Copy the object dimensions vector and shuffle on the copy
  objectDimensionsArray_shuffled = objectDimensionsArray;
  random_shuffle(objectDimensionsArray_shuffled.begin(), objectDimensionsArray_shuffled.end());

  artificialObjectDimensionsArray_shuffled = artificialObjectDimensionsArray;
  random_shuffle(artificialObjectDimensionsArray_shuffled.begin(), artificialObjectDimensionsArray_shuffled.end());

  realisticObjectDimensionsArray_shuffled = realisticObjectDimensionsArray;
  random_shuffle(realisticObjectDimensionsArray_shuffled.begin(), realisticObjectDimensionsArray_shuffled.end());

  //Construct the cell locations dynamic array
  spawnedObjectCenterLocations = new cellLocation[currentIndex];

  return;
}

/*******************************/
//The function to spawn objects
void SimulationWorld::spawnObject(string objectName, float x, float y, float yaw, float z)
{
  stringstream spawnCommand;

  //Reset the string stream
  spawnCommand.str("");

  //Construct the command stream
  spawnCommand << ". /opt/ros/kinetic/setup.sh; rosrun gazebo_ros spawn_model -database " << objectName << " -sdf -model " << objectName << " -x " << x << " -y " << y << " -z " << z << " -Y " << yaw << " > /dev/null";

  //Spawn the model
  system(spawnCommand.str().c_str());

  return;
}

/*******************************/
//Function to delete an object
void SimulationWorld::deleteObject(string objectName)
{
  stringstream deleteCommand;

  //Reset the string stream
  deleteCommand.str("");

  //Construct the command stream
  deleteCommand << ". /opt/ros/kinetic/setup.sh; rosservice call gazebo/delete_model '{model_name: " << objectName << "}'" << " > /dev/null";

  //Delete the model
  system(deleteCommand.str().c_str());

  return;
}

/*******************************/
//The function to create a subprocess for spawning and deleting Gazebo models and terminate that if Gazebo is not responsive withing a time frame
void SimulationWorld::callGazebo(const string command)
{
  system(command.c_str());

  /* The mechanism to check if a Gazebo instruction is finished, and if it takes more time than expected it is cancelled
   * This is removed, because it is too slow due to the creation of a process each time

  const int GAZEBO_TIMEOUT = 60;

  //Declare a counter and a process status variable
  int waitCounter = 0, status;

  //Fork a process
  pid_t gazeboChildProcess = fork();

  //Check the process is the child or the parent
  if(gazeboChildProcess == 0) //Child
    {
      //Replace the child the command
      execlp("bash", "bash", "-c", command.c_str(), NULL);

      //Print an error message and quit if execlp failes
      fprintf(stderr, "\nError in execlp\n");
      exit(EXIT_FAILURE);
    }
  else if(gazeboChildProcess > 0) //Parent
    {
      //Repeatedly check for child process to finish
      while(true)
	{
	  //Non-blocking wait for the child process to finish
	  waitpid(gazeboChildProcess, & status, WNOHANG);

	  //Sleep 1 ms
	  usleep(1000);
	  waitCounter++;

	  //If the child process is finished break out of the loop
	  if(WIFEXITED(status))
	    break;

	  //If time is over, terminate the child process and break the loop
	  if(waitCounter > (GAZEBO_TIMEOUT * 1000))
	    {
	      kill(gazeboChildProcess, SIGTERM);
	      waitpid(gazeboChildProcess, NULL, (int) NULL);
	      break;
	    }
	}
    }
  else if(gazeboChildProcess < 0) //Error happend
    {
      perror("Error in opening a new process");
      exit(EXIT_FAILURE);
    }
*/

  return;
}

/*******************************/
//Function to move an object
void SimulationWorld::moveObject(string objectName, float x, float y, float yaw, float z)
{
  //Reset the string stream
  command.str("");

  //Convert from RPY to Quaternion
  tf::Quaternion convertedQuaternion = tf::createQuaternionFromRPY(0, 0, yaw);

  //Construct the command stream
  command << ". /opt/ros/kinetic/setup.sh; rosservice call /gazebo/set_model_state '{model_state: { model_name: " << objectName <<", pose: { position: { x: " << x << ", y: "<< y << ", z: " << z << " }, orientation: {x: " << convertedQuaternion.x() << ", y: " << convertedQuaternion.y() << ", z: " << convertedQuaternion.z() << ", w: " << convertedQuaternion.w() << " } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'" <<  " > /dev/null";

  //Run the command
  moveThread.push_back(thread(& SimulationWorld::callGazebo, this, command.str()));

  return;
}

/*******************************/
//Function to move an object back to its initial location
void SimulationWorld::moveBack(objectID object_ID)
{
  //Extract the object's info
  moveBack_objectName = object_ID.name;
  moveBack_x = initialObjectLocations[object_ID.index].x;
  moveBack_y = initialObjectLocations[object_ID.index].y;

  //Reset the string stream
  command.str("");

  //Convert from RPY to Quaternion
  tf::Quaternion convertedQuaternion = tf::createQuaternionFromRPY(0, 0, 0);

  //Construct the command stream
  command << ". /opt/ros/kinetic/setup.sh; rosservice call /gazebo/set_model_state '{model_state: { model_name: " << moveBack_objectName <<", pose: { position: { x: " << moveBack_x << ", y: "<< moveBack_y << ", z: " << 0 << " }, orientation: {x: " << convertedQuaternion.x() << ", y: " << convertedQuaternion.y() << ", z: " << convertedQuaternion.z() << ", w: " << convertedQuaternion.w() << " } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'" <<  " > /dev/null";

  //Run the command
  moveBackThread.push_back(thread(& SimulationWorld::callGazebo, this, command.str()));

  return;
}

/*******************************/
//Function to spawn objects in a distant place from the robot
void SimulationWorld::initialSpawning()
{
  //Set maximum number of objects in a row
  const int maxInRow = 10;

  //Save the number of available objects
  int numberOfObjects = objectDimensionsArray.size();

  //Compute the maximum available dimension in the list of objects
  float maxDimension = max(maxLength, maxWidth);

  //Define the distance between objects
  float distance = 1.5 * maxDimension;

  //Set the initial placement location
  float x = -8 - (ceil(numberOfObjects / (float) maxInRow) * distance), y = -9 - (maxInRow * distance), intialY;
  intialY = y;

  //Clear the initial object locations array
  initialObjectLocations.clear();

  //Initialize the number of objects in the current row to zero
  int objectsInRow = 0;

  //Spawn all the objects
  for(int i = 0; i < numberOfObjects; i++)
    {
      //Spawn the object
      //spawnObject(objectDimensionsArray[i].name, x, y, 0, ground_z);
      spawnThread.push_back(thread(& SimulationWorld::spawnObject, this, objectDimensionsArray[i].name, x, y, 0, ground_z));

      //Store the location in the array
      currentLocation.x = x;
      currentLocation.y =y;
      initialObjectLocations.push_back(currentLocation);

      //Update the placement location for the next object
      y += distance;
      objectsInRow++;
      if(objectsInRow >= 10)
	{
	  objectsInRow = 0;
	  y = intialY;
	  x += distance;
	}
    }

  //Join the spawning threads
  for(threadCounter = 0; threadCounter < spawnThread.size(); threadCounter++)
    spawnThread[threadCounter].join();

  //Clear the vector of threads
  spawnThread.clear();

  return;
}

