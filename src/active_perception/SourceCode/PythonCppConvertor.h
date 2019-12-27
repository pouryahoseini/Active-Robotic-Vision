/*
 * PythonC++Converter.h
 *
 *  Created on: Oct 3, 2017
 *      Author: pourya
 */

//Guard
#ifndef SRC_PYTHONCPPCONVERTOR_H_
#define SRC_PYTHONCPPCONVERTOR_H_

//Headers
#include <cstdio>
#include <cstdlib>

#include <iostream>

#include "opencv2/opencv.hpp"

#include <Python.h>
#include <boost/python.hpp>
#include <boost/numpy.hpp>

//Global definitions
#include "GlobalDefinitions.h"

//Namespaces
using namespace std;
using namespace cv;
namespace bp = boost::python;
namespace np = boost::numpy;

//Class to convert C++ and Python data types
template<class inputDataType>
class Python_Cpp_Convertor
{
public:

  //Dummy constructor
  Python_Cpp_Convertor() {}

  //C++ OpenCV Mat to Python ndarray convertor (for 2D arrays)
  np::ndarray mat_to_ndarray_2d(Mat & inputMat);

  //Python ndarray to C++ OpenCV CV_64F Mat (for 2D arrays)
  Mat ndarray_to_mat_2d(np::ndarray & inputNdarray);

private:
  size_t elem_step;
  inputDataType *pointerToMatData, *pointerToNdarrayData;
  bp::tuple ndarray_shape, ndarray_stride;
  Py_intptr_t const *ndarrayShape;
};

//Explicit instantiations of the template class
template class Python_Cpp_Convertor<double>;
template class Python_Cpp_Convertor<float>;
template class Python_Cpp_Convertor<int>;

#endif /* SRC_PYTHONCPPCONVERTOR_H_ */
