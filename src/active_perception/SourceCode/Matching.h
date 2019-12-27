/*
 * Matching.h
 *
 *  Created on: Oct 13, 2017
 *      Author: pourya
 */

//Guard
#ifndef SOURCECODE_MATCHING_H_
#define SOURCECODE_MATCHING_H_

//Declarations
#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <cfloat>
#include <vector>
#include <numeric>

#include <opencv2/opencv.hpp>

//Global definitions
#include "GlobalDefinitions.h"

//Namespaces being used
using namespace std;
using namespace cv;

//The class to perform matching in images
class Matching
{
public:
  //Constructor
  Matching(int Image_Height = IMAGE_HEIGHT, int Image_Width = IMAGE_WIDTH, MatchingTypes matchingType= MatchingTypes::ShortestDistance);

  /*Function to perform matching between a set of bounding boxes and a set of points.
   *It stores the matches in a vector of Vec2i pairs. The first element in the pair is the bounding box index and the second element is the centroid index.
   *It It returns true if any match is found, else returns false.*/
  bool Match(vector<Rect2d> & secondaryView_boundingBoxes, vector<Point> & mainView_covertedCentroids, vector<Vec2i> & output_listOfMatches);

  //Function to save the associations between the main view and the secondary view
  void saveMatches(vector<Vec2i> & input_listOfMatches, vector<objectInfo> & mainViewObjects, vector<objectInfo> & secondaryViewObjects);

private:
  /*The function to match bounding boxes with points in an image
   * The output vector of Veci2 (output_listOfMatches) stores index of the matched point in the second column, while the first column is the index of the matched bounding box.
   * The function returns true if it finds any matches, and returns false without changing output_listOfMatches if no match found*/
  bool boundingBoxesToPointsGeometricMatcher(vector<Rect2d> & secondaryView_boundingBoxes, vector<Point> & mainView_covertedCentroids, vector<Vec2i> & output_listOfMatches);

  //The function to calculate the eight surrounding points of a rectangle and save them into the eight responsible variables defined in this class
  vector<Point> surroundingPointsExtractor(Rect2d & secondaryView_boundingBoxes);

  int searchingCategoryNumber, queriedCategoryNumber, searchingCounter, queriedCounter, searchingStartPoint, queriedStartPoint, ROI_Top, ROI_Left, ROI_Width, ROI_Height, eightPointsCounter;
  int associatedQueriedCategory, rowNumber, rowCounter, maximumDistance, numberOfBoundingBoxes, numberOfPoints, pairCounter;
  unsigned char distanceCounter;
  bool pointsAsSearchingComponent;
  double euclideanDistance[8], * minimumOfEightDistances, minimumDistance, averageDistance, standardDeviationOfDistances, ratioOfStandardDeviation;
  vector<Point> * allEightPointsMatrix;
  vector<Point> eightPoints = vector<Point>(8);
  vector<int> listOfMatchesWithQueried;
  vector<double> listOfDistances, listOfDistancesWithQueried;
  vector<Vec2i> listOfMatches, refinedListOfMatches;
  bool (Matching::* matchingFunction_ptr)(vector<Rect2d> &, vector<Point> &, vector<Vec2i> &);
};

#endif /* SOURCECODE_MATCHING_H_ */
