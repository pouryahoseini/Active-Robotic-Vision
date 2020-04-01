/*
 * Matching.cpp
 *
 *  Created on: Oct 13, 2017
 *      Author: pourya
 */

//Declaration
#include "Matching.h"

/*******************************/
//Constructor
Matching::Matching(int Image_Height, int Image_Width, MatchingTypes matchingType)
{
  //set the ratio of standard deviation of normal distribution of distances in the distance-based matching
  ratioOfStandardDeviation = STANDARD_DEVIATION_RATIO;

  //set the maximum acceptable distance between two associated objects
  maximumDistance = 10* min(Image_Height, Image_Width) / MAXIMUM_MATCHING_DISTANCE_DENOMINATOR;

  //Decide on the matching function to use
  if(matchingType == MatchingTypes::ShortestDistance)
    matchingFunction_ptr = & Matching::boundingBoxesToPointsGeometricMatcher;
}

/*******************************/
/*Function to perform matching between a set of bounding boxes and a set of points.
 *It stores the matches in a vector of Vec2i pairs. The first element in the pair is the bounding box index and the second element is the centroid index.
 *It It returns true if any match is found, else returns false.*/
bool Matching::Match(vector<Rect2d> & secondaryView_boundingBoxes, vector<Point> & mainView_covertedCentroids, vector<Vec2i> & output_listOfMatches)
{
  return (this->* matchingFunction_ptr)(secondaryView_boundingBoxes, mainView_covertedCentroids, output_listOfMatches);
}

/*******************************/
/*The function to match bounding boxes with points in an image
 * The output vector of Veci2 (output_listOfMatches) stores index of the matched point in the second column, while the first column is the index of the matched bounding box.
 * The function returns true if it finds any matches, and returns false without changing output_listOfMatches if no match found*/
bool Matching::boundingBoxesToPointsGeometricMatcher(vector<Rect2d> & secondaryView_boundingBoxes, vector<Point> & mainView_covertedCentroids, vector<Vec2i> & output_listOfMatches)
{
  //clear the list of matches
  listOfMatches.clear();
  listOfDistances.clear();
  listOfMatchesWithQueried.clear();
  listOfDistancesWithQueried.clear();
  refinedListOfMatches.clear();

  //Get the number of bounding boxes and the centroid points
  numberOfBoundingBoxes = secondaryView_boundingBoxes.size();
  numberOfPoints = mainView_covertedCentroids.size();

  //setting row number to -1
  rowNumber = -1;

  //check which category has fewer objects, and set it as "searching" category; the other one as "queried" category
  searchingCategoryNumber = min(numberOfBoundingBoxes, numberOfPoints);
  queriedCategoryNumber = max(numberOfBoundingBoxes, numberOfPoints);

  //returning false, because at least one of the categories has no objects
  if (! (searchingCategoryNumber >= 1))
    return false;

  //setting the starting point to look in input arrays
  if (searchingCategoryNumber == numberOfPoints)
    pointsAsSearchingComponent = true;
  else
    pointsAsSearchingComponent = false;

  //setting start points for counting through the input lists
  searchingStartPoint = 0;
  queriedStartPoint = 0;

  //constructing the list of matched searching objects with queried objects
  listOfMatchesWithQueried.resize(queriedCategoryNumber);
  listOfDistancesWithQueried.assign(queriedCategoryNumber, DBL_MAX);

  //find the four corners of all the bounding boxes and also the four middle points of their edges
  allEightPointsMatrix = new vector<Point>[numberOfBoundingBoxes * 8];
  for (eightPointsCounter = 0; eightPointsCounter < numberOfBoundingBoxes; eightPointsCounter++)
    allEightPointsMatrix[eightPointsCounter] = surroundingPointsExtractor(secondaryView_boundingBoxes[eightPointsCounter]);

  //for each searching category do:
  for (searchingCounter = searchingStartPoint; searchingCounter < (searchingCategoryNumber + searchingStartPoint); searchingCounter++)
    {
      //setting the minimum distance to the largest possible value
      minimumDistance = DBL_MAX;

      //for each "queried" category do:
      for (queriedCounter = queriedStartPoint; queriedCounter < (queriedCategoryNumber + queriedStartPoint); queriedCounter++)
	{
	  //finding the Euclidean distances between the input point and the eight corners of the bounding box
	  if (pointsAsSearchingComponent == true)
	      {
	      for (distanceCounter = 0; distanceCounter < 8; distanceCounter++)
		euclideanDistance[distanceCounter] = norm(allEightPointsMatrix[queriedCounter][distanceCounter] - mainView_covertedCentroids[searchingCounter]);
	      }
	  else
	      {
	      for (distanceCounter = 0; distanceCounter < 8; distanceCounter++)
		euclideanDistance[distanceCounter] = norm(allEightPointsMatrix[searchingCounter][distanceCounter] - mainView_covertedCentroids[queriedCounter]);
	      }

          //find the corner/middle point closest to the input point
	  minimumOfEightDistances = min_element(euclideanDistance, euclideanDistance + 8);

	  //find the matched queried category with the searching category by taking the one with minimum distance
	  if ((* minimumOfEightDistances) < minimumDistance)
	    {
	      minimumDistance = (* minimumOfEightDistances);
	      associatedQueriedCategory = queriedCounter;
	    }
	}

      /*The following if statement saves the associations and their corresponding distances.
       * If there is a former association with the queried component if only saves that match if its corresponding distance is less than the former one
       If that's the case, it overwrites the previous association with larger distance with current one*/

      //check if there is no former association with the queried object, save the associations in the record
      if (listOfDistancesWithQueried[associatedQueriedCategory] == DBL_MAX)
	{
	  //save the distance to the list of associated distances to the queried object
	  listOfDistancesWithQueried[associatedQueriedCategory] = minimumDistance;

	  //save the association to the list of matches for the queried object
	  rowNumber++;
	  listOfMatchesWithQueried[associatedQueriedCategory] = rowNumber;

	  //save the distance to the list of distances
	  listOfDistances.push_back(minimumDistance);

	    //save the association to the list of matches
	  if (pointsAsSearchingComponent == true)
	    listOfMatches.push_back(Vec2i(associatedQueriedCategory, searchingCounter));
	  else
	    listOfMatches.push_back(Vec2i(searchingCounter, associatedQueriedCategory));

	}
      //check if the current matching distance for queried object is less than any of its previously matched distances
      else if ( minimumDistance < listOfDistancesWithQueried[associatedQueriedCategory])
	{
	  //setting the new minimum matching distance for the queried component
	  listOfDistancesWithQueried[associatedQueriedCategory] = minimumDistance;

	  //replace the association with larger distance to the current queried object in the matching list
	  if (pointsAsSearchingComponent == true)
	    listOfMatches[listOfMatchesWithQueried[associatedQueriedCategory]] = Vec2i(associatedQueriedCategory, searchingCounter);
	  else
	    listOfMatches[listOfMatchesWithQueried[associatedQueriedCategory]] = Vec2i(searchingCounter, associatedQueriedCategory);

	  //save the distance to the list of distances
	  listOfDistances[listOfMatchesWithQueried[associatedQueriedCategory]] = minimumDistance;

	}
    }

  //delete the dynamic array
  delete [] allEightPointsMatrix;

  //compute the average and standard deviation of the distances
  averageDistance = std::accumulate(listOfDistances.begin(), listOfDistances.end(), 0.0) / static_cast<double>(listOfDistances.size());
  for (rowCounter = 0; rowCounter < listOfDistances.size(); rowCounter++)
    standardDeviationOfDistances += pow(listOfDistances[rowCounter] - averageDistance, 2.0);
  standardDeviationOfDistances /= static_cast<double>(listOfDistances.size());
  standardDeviationOfDistances = sqrt(standardDeviationOfDistances);

  //remove the associations with very low or high values in the assumed and estimated Gaussian model
  for (rowCounter = 0; rowCounter < listOfDistances.size(); rowCounter++)
    if ( (! (listOfDistances[rowCounter] > (ratioOfStandardDeviation * standardDeviationOfDistances + averageDistance))) && (! (listOfDistances[rowCounter] > maximumDistance)) )
      refinedListOfMatches.push_back(listOfMatches[rowCounter]);

  //returning true, if a match is found
  if (refinedListOfMatches.empty())
    return false;
  else
    {
      //save the output vector
      output_listOfMatches = refinedListOfMatches;

      return true;
    }

}

/*******************************/
//The function to calculate the eight surrounding points of a rectangle and save them into the eight responsible variables defined in this class
//Note: I assumed y axis as height and x axis as width in agreement with OpenCV convention
vector<Point>  Matching::surroundingPointsExtractor(Rect2d & secondaryView_boundingBoxes)
{
  //extracting region of interest information
  ROI_Top = secondaryView_boundingBoxes.y;
  ROI_Left = secondaryView_boundingBoxes.x;
  ROI_Width = secondaryView_boundingBoxes.width;
  ROI_Height = secondaryView_boundingBoxes.height;

  //computing the eight surrounding points locations
  eightPoints[0].x = ROI_Left; // = topLeft.x
  eightPoints[0].y = ROI_Top; // = topLeft.y
  eightPoints[1].x = ROI_Left + (ROI_Width / 2.0); // = topCenter.x
  eightPoints[1].y = ROI_Top; // = topCenter.y
  eightPoints[2].x = ROI_Left + ROI_Width; // = topRight.x
  eightPoints[2].y = ROI_Top; // = topRight.y
  eightPoints[3].x = ROI_Left; // = middleLeft.x
  eightPoints[3].y = ROI_Top + (ROI_Height / 2.0); // = middleLeft.y
  eightPoints[4].x = ROI_Left + ROI_Width; // = middleRight.x
  eightPoints[4].y = ROI_Top + (ROI_Height / 2.0); // = middleRight.y
  eightPoints[5].x = ROI_Left; // = bottomLeft.x
  eightPoints[5].y = ROI_Top + ROI_Height; // = bottomLeft.y
  eightPoints[6].x = ROI_Left + ROI_Width; // = bottomRight.x
  eightPoints[6].y = ROI_Top + ROI_Height; // = bottomRight.y
  eightPoints[7].x = ROI_Left + (ROI_Width / 2.0); // = bottomCenter.x
  eightPoints[7].y = ROI_Top + ROI_Height; // = bottomCenter.y

  return eightPoints;
}

/*******************************/
//Function to save the associations between the main view and the secondary view
void Matching::saveMatches(vector<Vec2i> & input_listOfMatches, vector<objectInfo> & mainViewObjects, vector<objectInfo> & secondaryViewObjects)
{
  //For all the matching pairs
  for(pairCounter = 0; pairCounter < input_listOfMatches.size(); pairCounter++)
    {
      //Update the matched main view object
      mainViewObjects[input_listOfMatches[pairCounter].val[1]].matchedIndex = input_listOfMatches[pairCounter].val[0];
      mainViewObjects[input_listOfMatches[pairCounter].val[1]].matchFound = true;

      //Update the matched secondary view object
      secondaryViewObjects[input_listOfMatches[pairCounter].val[0]].matchedIndex = input_listOfMatches[pairCounter].val[1];
      secondaryViewObjects[input_listOfMatches[pairCounter].val[0]].matchFound = true;
    }

  return;
}


