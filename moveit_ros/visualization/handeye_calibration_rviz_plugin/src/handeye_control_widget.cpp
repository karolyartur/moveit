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

#include <moveit/handeye_calibration_rviz_plugin/handeye_control_widget.h>

namespace moveit_rviz_plugin
{
ControlTabWidget::ControlTabWidget(QWidget* parent)
  : QWidget(parent), tf_listener_(tf_buffer_), sensor_mount_type_(EYE_TO_HAND)
{
  QVBoxLayout* layout = new QVBoxLayout();
  this->setLayout(layout);

  QHBoxLayout* calib_layout = new QHBoxLayout();
  layout->addLayout(calib_layout);

  // Auto calibrate progress
  auto_progress_ = new QProgressBar();
  auto_progress_->setTextVisible(true);
  layout->addWidget(auto_progress_);

  // Pose sample tree view area
  QGroupBox* sample_group = new QGroupBox("Pose_sample");
  sample_group->setFixedWidth(280);
  calib_layout->addWidget(sample_group);
  QVBoxLayout* sample_layout = new QVBoxLayout();
  sample_group->setLayout(sample_layout);

  sample_tree_view_ = new QTreeView(this);
  sample_tree_view_->setAutoScroll(true);
  sample_tree_view_->setAlternatingRowColors(true);
  tree_view_model_ = new QStandardItemModel(sample_tree_view_);
  sample_tree_view_->setModel(tree_view_model_);
  sample_tree_view_->setHeaderHidden(true);
  sample_tree_view_->setIndentation(10);
  sample_layout->addWidget(sample_tree_view_);

  // Setting area
  QVBoxLayout* layout_right = new QVBoxLayout();
  calib_layout->addLayout(layout_right);

  QGroupBox* setting_group = new QGroupBox("Setting");
  layout_right->addWidget(setting_group);
  QFormLayout* setting_layout = new QFormLayout();
  setting_group->setLayout(setting_layout);

  calibration_solver_ = new QComboBox();
  setting_layout->addRow("AX=XB Solver", calibration_solver_);

  load_joint_state_btn_ = new QPushButton("Load Joint States");
  setting_layout->addRow(load_joint_state_btn_);

  save_joint_state_btn_ = new QPushButton("Save Joint states");
  setting_layout->addRow(save_joint_state_btn_);

  save_camera_pose_btn_ = new QPushButton("Save Camera Pose");
  setting_layout->addRow(save_camera_pose_btn_);

  // Manual calibration area
  QGroupBox* manual_cal_group = new QGroupBox("Manual Calibration");
  layout_right->addWidget(manual_cal_group);
  QHBoxLayout* control_cal_layout = new QHBoxLayout();
  manual_cal_group->setLayout(control_cal_layout);

  take_sample_btn_ = new QPushButton("Take Sample");
  take_sample_btn_->setFixedHeight(50);
  connect(take_sample_btn_, SIGNAL(clicked(bool)), this, SLOT(takeSampleBtnClicked(bool)));
  control_cal_layout->addWidget(take_sample_btn_);

  reset_sample_btn_ = new QPushButton("Reset Sample");
  reset_sample_btn_->setFixedHeight(50);
  control_cal_layout->addWidget(reset_sample_btn_);

  // Auto calibration area
  QGroupBox* auto_cal_group = new QGroupBox("Auto Calibration");
  layout_right->addWidget(auto_cal_group);
  QHBoxLayout* auto_cal_layout = new QHBoxLayout();
  auto_cal_group->setLayout(auto_cal_layout);

  start_auto_calib_btn_ = new QPushButton("Start");
  start_auto_calib_btn_->setFixedHeight(50);
  start_auto_calib_btn_->setToolTip("Start or resume auto calibration process");
  auto_cal_layout->addWidget(start_auto_calib_btn_);

  pause_auto_calib_btn_ = new QPushButton("Pause");
  pause_auto_calib_btn_->setFixedHeight(50);
  pause_auto_calib_btn_->setToolTip("Pause the auto calibration process");
  auto_cal_layout->addWidget(pause_auto_calib_btn_);

  skip_robot_state_btn_ = new QPushButton("Skip");
  skip_robot_state_btn_->setFixedHeight(50);
  skip_robot_state_btn_->setToolTip("Skip the current robot state target");
  auto_cal_layout->addWidget(skip_robot_state_btn_);
}

void ControlTabWidget::loadWidget(const rviz::Config& config)
{

}

void ControlTabWidget::saveWidget(rviz::Config& config)
{

}

void ControlTabWidget::setTFTool(rviz_visual_tools::TFVisualToolsPtr& tf_pub)
{
  tf_tools_ = tf_pub;
}

void ControlTabWidget::addPoseSampleToTreeView(const geometry_msgs::TransformStamped& cTo, 
                                               const geometry_msgs::TransformStamped& bTe, int id)
{

  std::string item_name = "Sample " + std::to_string(id);
  QStandardItem* parent = new QStandardItem(QString(item_name.c_str()));
  tree_view_model_->appendRow(parent);

  QStandardItem* child_1 = new QStandardItem("bTe");
  QStandardItem* child_1_1 = new QStandardItem("T");
  child_1_1->appendRow(new QStandardItem(("x: " + std::to_string(bTe.transform.translation.x)).c_str()));
  child_1_1->appendRow(new QStandardItem(("y: " + std::to_string(bTe.transform.translation.y)).c_str()));
  child_1_1->appendRow(new QStandardItem(("z: " + std::to_string(bTe.transform.translation.z)).c_str()));
  QStandardItem* child_1_2 = new QStandardItem("R");
  child_1_2->appendRow(new QStandardItem(("x: " + std::to_string(bTe.transform.rotation.x)).c_str()));
  child_1_2->appendRow(new QStandardItem(("y: " + std::to_string(bTe.transform.rotation.y)).c_str()));
  child_1_2->appendRow(new QStandardItem(("z: " + std::to_string(bTe.transform.rotation.z)).c_str()));
  child_1_2->appendRow(new QStandardItem(("w: " + std::to_string(bTe.transform.rotation.w)).c_str()));
  child_1->appendRow(child_1_1);
  child_1->appendRow(child_1_2);
  parent->appendRow(child_1);

  QStandardItem* child_2 = new QStandardItem("cTo");
  QStandardItem* child_2_1 = new QStandardItem("T");
  child_2_1->appendRow(new QStandardItem(("x: " + std::to_string(cTo.transform.translation.x)).c_str()));
  child_2_1->appendRow(new QStandardItem(("y: " + std::to_string(cTo.transform.translation.y)).c_str()));
  child_2_1->appendRow(new QStandardItem(("z: " + std::to_string(cTo.transform.translation.z)).c_str()));
  QStandardItem* child_2_2 = new QStandardItem("R");
  child_2_2->appendRow(new QStandardItem(("x: " + std::to_string(cTo.transform.rotation.x)).c_str()));
  child_2_2->appendRow(new QStandardItem(("y: " + std::to_string(cTo.transform.rotation.y)).c_str()));
  child_2_2->appendRow(new QStandardItem(("z: " + std::to_string(cTo.transform.rotation.z)).c_str()));
  child_2_2->appendRow(new QStandardItem(("w: " + std::to_string(cTo.transform.rotation.w)).c_str()));
  child_2->appendRow(child_2_1);
  child_2->appendRow(child_2_2);
  parent->appendRow(child_2);
}

void ControlTabWidget::UpdateSensorMountType(QString setup)
{
  if (setup.compare("Eye-to-Hand"))
    sensor_mount_type_ = EYE_TO_HAND;

  if (setup.compare("Eye-in-Hand"))
    sensor_mount_type_ = EYE_IN_HAND;

  ROS_DEBUG_STREAM("Sensor mount type changed: " << setup.toStdString());
}

void ControlTabWidget::updateFrameNames(std::map<std::string, std::string> names)
{
  frame_names_ = names;
  for (const std::pair<const std::string, std::string>& name : names)
    ROS_DEBUG_STREAM(name.first << " : " << name.second);
}

void ControlTabWidget::takeSampleBtnClicked(bool clicked)
{

  geometry_msgs::TransformStamped cTo;
  geometry_msgs::TransformStamped bTe;
  try
  {
    // Get the transform of the object w.r.t the camera
    cTo = tf_buffer_.lookupTransform(frame_names_["sensor"], frame_names_["object"], ros::Time(0));

    // Get the transform of the end-effector w.r.t the robot base
    bTe = tf_buffer_.lookupTransform(frame_names_["base"], frame_names_["eef"], ros::Time(0));
  }
  catch (tf2::TransformException& e)
  {
    ROS_WARN("TF exception: %s", e.what());
    return;
  }

  // save the pose samples
  effector_wrt_world_.push_back(tf2::transformToEigen(bTe));
  object_wrt_sensor_.push_back(tf2::transformToEigen(cTo));

  ControlTabWidget::addPoseSampleToTreeView(cTo, bTe, effector_wrt_world_.size());
}

} // namespace moveit_rviz_plugin