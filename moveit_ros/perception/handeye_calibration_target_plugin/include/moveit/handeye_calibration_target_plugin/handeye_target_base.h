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

#ifndef MOVEIT_HANDEYE_TARGET_BASE_
#define MOVEIT_HANDEYE_TARGET_BASE_

#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>

namespace moveit_handeye_calibration
{
class HandEyeTargetBase
{
public:

  virtual ~HandEyeTargetBase() = default;
  HandEyeTargetBase()
  {
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    distor_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
  }

  virtual bool initialize() = 0;

  virtual bool setTargetIntrinsicParams(const int& markers_x, const int& markers_y, 
                                        const int& marker_size_, const int& separation, 
                                        const int& border_bits, const std::string& dictionary_id) = 0;


  virtual bool setTargetDimension(const double& marker_size, const double& marker_seperation) = 0;

  virtual bool createTargetImage(cv::Mat& image) = 0;

  virtual bool detectTargetPose(cv::Mat& image) = 0;

  virtual std::vector<std::string> getDictionaryIds() = 0;

  virtual geometry_msgs::TransformStamped getPose(std::string& frame_id) = 0;

  virtual bool setCameraIntrinsicParams(const sensor_msgs::CameraInfoPtr& msg)
  {
    if (msg)
    {
      // Store camera matrix info
      for (size_t i = 0; i < 3; i++) 
      {
        for (size_t j = 0; j < 3; j++) 
        {
          camera_matrix_.at<double>(i, j) = msg->K[i*3+j];
        }
      }

      // Store camera distortion info
      for (size_t i = 0; i < 5; i++) 
      {
        distor_coeffs_.at<double>(i,0) = msg->D[i];
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("handeye_target_base", "CameraInfo msgs is NULL.");
    }
  }
  
protected:

  // 3x3 floating-point camera matrix
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  cv::Mat camera_matrix_;

  // Vector of distortion coefficients (k1, k2, t1, t2, k3)
  // Assume `plumb_bob` model
  cv::Mat distor_coeffs_;

};
}  // namespace moveit_handeye_calibration
#endif