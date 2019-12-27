/*
 * PythonC++Convertor.cpp
 *
 *  Created on: Oct 3, 2017
 *      Author: pourya
 */

//Header file
#include "PythonCppConvertor.h"


//C++ OpenCV Mat to Python ndarray convertor (for 2D arrays)
template<class inputDataType>
np::ndarray Python_Cpp_Convertor<inputDataType>::mat_to_ndarray_2d(Mat & inputMat)
{
  //getting the pointer to Mat data
  pointerToMatData = (inputDataType *) inputMat.data;
  elem_step = inputMat.step / sizeof(inputDataType);

  //setting conversion parameters
  np::dtype ndarray_dtype = np::dtype::get_builtin<inputDataType>();
  ndarray_shape = bp::make_tuple(inputMat.rows, inputMat.cols);
  ndarray_stride = bp::make_tuple(elem_step * sizeof(inputDataType), sizeof(inputDataType));

  //converting the mat type to a 2d ndarray
  np::ndarray py_ndarray = np::from_data(pointerToMatData, ndarray_dtype, ndarray_shape, ndarray_stride, bp::object());

  //return the ndarray object
  return py_ndarray;
}

//Python ndarray to C++ OpenCV CV_64F Mat (for 2D arrays)
template<>
Mat Python_Cpp_Convertor<double>::ndarray_to_mat_2d(np::ndarray & inputNdarray)
{
  //getting the pointer to ndarray data
  pointerToNdarrayData = reinterpret_cast<double *>(inputNdarray.get_data());

  //getting the ndarray's shape
  ndarrayShape = inputNdarray.get_shape();

  //return the OpenCV Mat
  return Mat(ndarrayShape[0], ndarrayShape[1], CV_64F, pointerToNdarrayData);
}
