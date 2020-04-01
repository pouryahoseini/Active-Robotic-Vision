/*
 * Morphology.cpp
 *
 *  Created on: Feb 19, 2017
 *      Author: pourya
 */

//Headers
#include "Morphology.h"

//Constructor
Morphology::Morphology(Size imageSize)
{
  minDimension = min(imageSize.width, imageSize.height);
  kernel = getStructuringElement(MORPH_RECT, Size(static_cast<int>(structuringElementToImageRatio * minDimension), static_cast<int>(structuringElementToImageRatio * minDimension)));
}

//The function to compute opening by reconstruction
Mat & Morphology::OpeningByReconstruction(Mat & image)
{
  Mat erodedImage;
  erode(image, erodedImage, kernel, Point(-1, -1), numberOfErosionDilation);
  image = ReconstructionByDilation(erodedImage, image);
  return image;
}

//The function to compute closing by reconstruction
Mat & Morphology::ClosingByReconstruction(Mat & image)
{
  Mat dilatedImage;
  dilate(image, dilatedImage, kernel, Point(-1, -1), numberOfErosionDilation);
  image = ReconstructionByErosion(dilatedImage, image);
  return image;
}

//The function to compute reconstruction by erosion
Mat & Morphology::ReconstructionByErosion(Mat & inputImage, Mat & mask)
{
  bool flag = false;
  Mat processedImage2;

  processedImage = Mat::zeros(inputImage.size(), inputImage.type());
  while (countNonZero(inputImage != processedImage) != 0)
    {
      if (flag == true)
	  inputImage = processedImage.clone();
      else
	  flag = true;

      erode(inputImage, processedImage2, kernel);
      processedImage = processedImage2 | mask;
    }
  return inputImage;
}

//The function to compute reconstruction by dilation
Mat & Morphology::ReconstructionByDilation(Mat & inputImage, Mat & mask)
{
  bool flag = false;
  Mat processedImage2;

    processedImage = Mat::zeros(inputImage.size(), inputImage.type());
    while (!equal(inputImage.begin<uchar>(), inputImage.end<uchar>(), processedImage.begin<uchar>())) //(countNonZero(inputImage != processedImage) != 0)
      {
        if (flag == true)
  	  inputImage = processedImage.clone();
        else
  	  flag = true;

        dilate(inputImage, processedImage2, kernel);
        processedImage = processedImage2 & mask;
      }
    return processedImage;
  }
