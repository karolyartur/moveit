/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019,  Intel Corporation.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Yu Yan */

#include <moveit/handeye_calibration_solver/handeye_solver_default.h>
#include <Python.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

namespace moveit_handeye_calibration
{
const std::string LOGNAME = "handeye_solver_default";

void HandEyeSolverDefault::initialize()
{
  solver_names_ = {"Daniilidis1999", "ParkBryan1994", "TsaiLenz1989"};
}

std::vector<std::string>& HandEyeSolverDefault::getSolverNames()
{
  return solver_names_;
}

bool HandEyeSolverDefault::solve(std::vector<Eigen::Isometry3d>& effector_wrt_world, 
                                 std::vector<Eigen::Isometry3d>& object_wrt_sensor, SENSOR_MOUNT_TYPE setup)
{
  char program_name[7] = "python";
  Py_SetProgramName(program_name);
  static int run_times{0};
  if (run_times == 0) // Numpy can only be loaded once, otherwise will segfault
  {
    ++run_times;
    Py_Initialize();
    atexit(Py_Finalize);
  } 
  ROS_INFO_STREAM("Python C API start");
  if (_import_array() < 0)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error importing numpy: ");
    return false;
  }
  // import_array();
  PyObject *p_name, *p_module, *p_class, *p_instance, *p_func_add_sample, *p_func_solve; 
  PyObject *p_args, *p_value;
  p_name = PyString_FromString("handeye.calibrator");
  p_module = PyImport_Import(p_name);
  Py_DECREF(p_name);

  if (!p_module)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to load python module: " << "handeye.calibrator");
    Py_DECREF(p_module);
    PyErr_Print();
    return false;
  }

  p_class = PyObject_GetAttrString(p_module, "HandEyeCalibrator");
  Py_DECREF(p_module);
  if (!p_class || !PyCallable_Check(p_class))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't find \"HandEyeCalibrator\" python class");
    Py_DECREF(p_class);
    Py_DECREF(p_module);
    PyErr_Print();
    return false;
  }

  if (setup == EYE_TO_HAND)
    p_value = PyString_FromString("Fixed");
  else if (setup == EYE_IN_HAND)
    p_value = PyString_FromString("Moving");
  
  if (!p_value)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't creat sensor mount type python value");
    Py_DECREF(p_value);
    Py_DECREF(p_class);
    PyErr_Print();
    return false;
  }

  p_args = PyTuple_New(1);
  PyTuple_SetItem(p_args, 0, p_value);
  Py_DECREF(p_value);
  if (!p_args)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't build python arguments");
    Py_DECREF(p_args);
    Py_DECREF(p_class);
    PyErr_Print();
    return false;
  }

  p_instance = PyEval_CallObject(p_class, p_args);
  Py_DECREF(p_args);
  Py_DECREF(p_class);
  if (!p_instance)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't create \"HandEyeCalibrator\" python instance");
    Py_DECREF(p_instance);
    PyErr_Print();
    return false;
  }

  if (effector_wrt_world.size() != object_wrt_sensor.size())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Sizes of two input pose samples don't match");
    Py_DECREF(p_instance);
    PyErr_Print();
    return false;
  }

  PyArrayObject *np_arg_eef_wrt_base, *np_arg_object_wrt_sensor;
  PyObject *p_array_eef_wrt_base, *p_array_object_wrt_sensor;
  npy_intp dims[2]{ARRAY_SIZE, ARRAY_SIZE};
  const int ND{2};
  double c_arr_eef_wrt_world[ARRAY_SIZE][ARRAY_SIZE];
  double c_arr_obj_wrt_sensor[ARRAY_SIZE][ARRAY_SIZE];
  size_t pose_num = effector_wrt_world.size();
  for (size_t i = 0; i < pose_num; ++i)
  {
    // Convert effector_wrt_world[i] from Eigen::isometry3d to PyArrayObject
    if (eigenIsometry3dToArray(effector_wrt_world[i], c_arr_eef_wrt_world))
    {
      // ROS_INFO_STREAM("Python C API debug \n");
      for (size_t i = 0; i < ARRAY_SIZE; ++i)
        for (size_t j = 0; j < ARRAY_SIZE; ++j)
          std::cout << c_arr_eef_wrt_world[i][j] << " ";

      p_array_eef_wrt_base = PyArray_SimpleNewFromData(ND, dims, NPY_DOUBLE, (void*)(c_arr_eef_wrt_world));
      // PyObject_Print(p_array, stdout, 0);
      if (!p_array_eef_wrt_base)
      {
        Py_DECREF(p_array_eef_wrt_base);
        Py_DECREF(p_instance);
        PyErr_Print();
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Error creating PyArray object");
        return false;
      }
      np_arg_eef_wrt_base = (PyArrayObject*)(p_array_eef_wrt_base);
      // Py_DECREF(p_array);
      if (PyArray_NDIM(np_arg_eef_wrt_base) == 2) // Check PyArrayObject dims are 4x4
      {
        npy_intp * py_array_dims = PyArray_DIMS(np_arg_eef_wrt_base);
        if ( py_array_dims[0] != 4 || py_array_dims[1] != 4)
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Error PyArrayObject dims: " << py_array_dims[0] << "x" << py_array_dims[1]);
          Py_DECREF(np_arg_eef_wrt_base);
          Py_DECREF(p_instance);
          return false;
        }
      }
    }

    // Convert object_wrt_sensor[i] from Eigen::isometry3d to PyArrayObject
    if (eigenIsometry3dToArray(object_wrt_sensor[i], c_arr_obj_wrt_sensor))
    {
      p_array_object_wrt_sensor = PyArray_SimpleNewFromData(ND, dims, NPY_DOUBLE, (void*)(c_arr_obj_wrt_sensor));
      if (!p_array_object_wrt_sensor)
      {
        Py_DECREF(p_array_object_wrt_sensor);
        Py_DECREF(p_instance);
        PyErr_Print();
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Error creating PyArray object");
        return false;
      }
      np_arg_object_wrt_sensor = (PyArrayObject*)(p_array_object_wrt_sensor);
      // Py_DECREF(p_array);
      if (PyArray_NDIM(np_arg_object_wrt_sensor) == 2) // Check PyArrayObject dims are 4x4
      {
        npy_intp * py_array_dims = PyArray_DIMS(np_arg_object_wrt_sensor);
        if ( py_array_dims[0] != 4 || py_array_dims[1] != 4)
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Error PyArrayObject dims: " << py_array_dims[0] << "x" << py_array_dims[1]);
          Py_DECREF(np_arg_object_wrt_sensor);
          Py_DECREF(p_instance);
          return false;
        }
      }
    }

    // long double* c_out = reinterpret_cast<long double*>(PyArray_DATA(p_array_object_wrt_sensor));
    for (size_t i = 0; i < 4; i++)
    {
      for (size_t j = 0; j < 4; j++)
          std::cout << *(double *)PyArray_GETPTR2(np_arg_eef_wrt_base, i, j) << " ";
      std::cout << std::endl;
    }

    for (size_t i = 0; i < 4; i++)
    {
      for (size_t j = 0; j < 4; j++)
          std::cout << *(double *)PyArray_GETPTR2(np_arg_object_wrt_sensor, i, j) << " ";
      std::cout << std::endl;
    }

    // Asign sample poses to 'HandEyeCalibrator' instance
    p_func_add_sample = PyObject_GetAttrString(p_instance, "add_sample");
    if (!p_func_add_sample || !PyCallable_Check(p_func_add_sample))
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't find 'add_sample' method");
      Py_DECREF(p_instance);
      PyErr_Print();
      return false;
    }
    PyObject* p_args_sample = Py_BuildValue("OO", np_arg_eef_wrt_base, np_arg_object_wrt_sensor);
    // Py_DECREF(np_arg_eef_wrt_base);
    // Py_DECREF(np_arg_object_wrt_sensor);
    if (!p_args_sample)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't create argument tuple for 'add_sample' method");
      Py_DECREF(p_instance);
      PyErr_Print();
      return false;
    }
    // Py_DECREF(np_arg_eef_wrt_base);
    // Py_DECREF(np_arg_object_wrt_sensor);
    p_value = PyEval_CallObject(p_func_add_sample, p_args_sample);
    Py_DECREF(p_args_sample);
    Py_DECREF(p_func_add_sample);
    // ROS_INFO_STREAM("Python C API debug \n");
    if (!p_value)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Error calling 'add_sample' method");
      // Py_DECREF(p_value);
      Py_DECREF(p_instance);
      PyErr_Print();
      return false;
    }
    // ROS_INFO_STREAM("Python C API debug \n");
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "num_samples: " << PyInt_AsLong(p_value));
  }
  Py_DECREF(p_instance);
  ROS_INFO_STREAM("Python C API end");
  return true;
}  

bool HandEyeSolverDefault::eigenIsometry3dToArray(const Eigen::Isometry3d& pose, double (* c_arr)[ARRAY_SIZE])
{
  const Eigen::MatrixXd& mat = pose.matrix();

  if (mat.rows() == ARRAY_SIZE && mat.cols() == ARRAY_SIZE)
  {
    for (size_t i = 0; i < ARRAY_SIZE; ++i)
      for (size_t j = 0; j < ARRAY_SIZE; ++j)
        c_arr[i][j] = mat(i, j);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error matrix dims: " << mat.rows() << "x" << mat.cols());
    return false;
  }
}


} // namespace moveit_handeye_calibration