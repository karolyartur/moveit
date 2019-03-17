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

#ifndef MOVEIT_HANDEYE_CALIBRATION_RVIZ_PLUGIN_HANDEYE_CALIBRATE_WIDGET_
#define MOVEIT_HANDEYE_CALIBRATION_RVIZ_PLUGIN_HANDEYE_CALIBRATE_WIDGET_

// qt
#include <QTreeView>
#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QProgressBar>
#include <QStandardItemModel>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

namespace moveit_rviz_plugin
{
class ControlTabWidget: public QWidget
{
  Q_OBJECT
public:
  
  explicit ControlTabWidget(QWidget* parent = Q_NULLPTR);
  ~ControlTabWidget()
  {
  }

  void loadWidget(const rviz::Config& config);
  void saveWidget(rviz::Config& config);

private Q_SLOTS:

Q_SIGNALS:

private:

  // ************************************************************** 
  // Qt components   
  // **************************************************************

  QTreeView* sample_tree_view_;
  QStandardItemModel* tree_view_model_;

  QComboBox* calibration_solver_;
  QPushButton* take_sample_btn_;
  QPushButton* reset_sample_btn_;

  QPushButton* start_auto_calib_btn_;
  QPushButton* skip_robot_state_btn_;

  // Load & save pose samples and joint goals
  QPushButton* save_joint_state_btn_;
  QPushButton* load_joint_state_btn_;
  QPushButton* save_camera_pose_btn_;

  QProgressBar* auto_progress_;

  // ************************************************************** 
  // Variables   
  // **************************************************************

  // ************************************************************** 
  // Ros components   
  // **************************************************************

};

} // namespace moveit_rviz_plugin
#endif