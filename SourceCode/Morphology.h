/*
 * Morphology.h
 *
 *  Created on: Feb 19, 2017
 *      Author: pourya
 */

//Guard
#ifndef MORPHOLOGY_H_
#define MORPHOLOGY_H_

//Headers
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//Global definitions
#include "GlobalDefinitions.h"

//Namespaces being used
using namespace std;
using namespace cv;

//The class used to perform morphological operations
class Morphology
{
public:
  //Constructor
  Morphology(Size imageSize = Size(330, 270));

  //Opening by reconstruction
  Mat & OpeningByReconstruction(Mat & image);

  //Closing by reconstruction
  Mat & ClosingByReconstruction(Mat & image);

private:
  Mat kernel;
  Mat & ReconstructionByErosion(Mat & inputImage, Mat & mask);
  Mat & ReconstructionByDilation(Mat & inputImage, Mat & mask);
  int minDimension;
  Mat processedImage;
  double structuringElementToImageRatio = STRUCTURING_ELEMENT_TO_IMAGE_RATIO;
  int numberOfErosionDilation = 2;
};


#endif /* MORPHOLOGY_H_ */
