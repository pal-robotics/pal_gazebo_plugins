/*
 *  gazebo_pal_hey5.cpp
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 6/3/2015
 *      Author: luca
 * \brief A plugin for gazebo for controlling the pal hey5 underactuated hand in simulation
 * \author  Luca Marchionni (luca.marchionni@pal-robotics.com)
 */

// Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <assert.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>

#include "ignition/math.hh"

#include "pal_gazebo_plugins/gazebo_underactuated_finger.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sdf/sdf.hh"


namespace gazebo
{

GazeboPalHey5::GazeboPalHey5() {}

// Destructor
GazeboPalHey5::~GazeboPalHey5()
{
}

// Load the controller
void GazeboPalHey5::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent = _parent;
  this->world = _parent->GetWorld();
  this->actuated_joint_name_ = "";
  this->robot_namespace_ = "";
  this->ros_node_ = gazebo_ros::Node::Get(_sdf);

  if (!_sdf->HasElement("namespace")) {
    RCLCPP_INFO_STREAM(
      ros_node_->get_logger(),
      "GazeboPalHey5 Plugin missing <namespace>, defaults to '" <<
        this->robot_namespace_ << "'");
  }

  if (!_sdf->HasElement("actuatedJoint")) {
    char error[200];
    snprintf(
      error, sizeof(error),
      "GazeboPalHey5 Plugin (ns = %s) missing actuatedJoint in urdf plugin description",
      this->robot_namespace_.c_str());
    gzthrow(error);
  } else {
    this->actuated_joint_name_ = _sdf->GetElement("actuatedJoint")->Get<std::string>();
  }

  if (!_sdf->HasElement("virtualJoint")) {
    char error[200];
    snprintf(
      error, sizeof(error),
      "GazeboPalHey5 Plugin (ns = %s) missing virtualJoint in urdf plugin description",
      this->robot_namespace_.c_str());
    gzthrow(error);
  }

  for (sdf::ElementPtr virtualJointPtr = _sdf->GetElement(std::string("virtualJoint"));
    virtualJointPtr;
    virtualJointPtr = _sdf->GetElement(std::string("virtualJoint")))
  {
    if (virtualJointPtr->HasElement("name")) {
      std::string virtualJointName = virtualJointPtr->GetElement("name")->Get<std::string>();
      virtual_joint_names_.push_back(virtualJointName);
    }

    if (virtualJointPtr->HasElement(std::string("scale_factor"))) {
      double scaleFactor = virtualJointPtr->GetElement(std::string("scale_factor"))->Get<double>();
      scale_factors_.push_back(scaleFactor);
    }

    if (virtualJointPtr->HasElement(std::string("pid_gains"))) {
      sdf::ElementPtr pidGainsPtr = virtualJointPtr->GetElement(std::string("pid_gains"));
      std::map<std::string, double> pidGains;

      for (std::string const & gain : {"p", "i", "d", "i_max", "i_min"}) {
        if (pidGainsPtr->HasElement(std::string(gain))) {
          pidGains.emplace(gain, pidGainsPtr->GetElement(std::string(gain))->Get<double>());
        }
      }

      pid_gains_.emplace_back(pidGains);
    } else {
      pid_gains_.emplace_back();
    }

    // Removing element to parse the next one, no better solution found
    virtualJointPtr->RemoveFromParent();
    if (!_sdf->HasElement("virtualJoint")) {
      break;
    }
  }

  gazebo::physics::JointPtr joint = this->parent->GetJoint(actuated_joint_name_);
  if (!joint) {
    char error[200];
    snprintf(
      error, sizeof(error),
      "GazeboPalHey5 Plugin (ns = %s) couldn't get actuated finger hinge joint named \"%s\"",
      this->robot_namespace_.c_str(), this->actuated_joint_name_.c_str());
    gzthrow(error);
  }
  actuated_joint_ = joint;
  actuator_angle_ = actuated_joint_->Position(0u);

  for (unsigned int i = 0; i < virtual_joint_names_.size(); ++i) {
    gazebo::physics::JointPtr joint_ptr = this->parent->GetJoint(virtual_joint_names_.at(i));

    if (!joint_ptr) {
      char error[200];
      snprintf(
        error, sizeof(error),
        "GazeboPalHey5 Plugin (ns = %s) couldn't get actuated finger hinge joint named \"%s\"",
        this->robot_namespace_.c_str(), this->virtual_joint_names_.at(i).c_str());
      gzthrow(error);
    }

    virtual_joints_.push_back(joint_ptr);

    /// @bug Why is this unstable if we place it directly in a shared pointer?
    auto pid = control_toolbox::PidROS(
      ros_node_,
      ros_node_->get_name() + std::string("/") + this->virtual_joint_names_.at(i));

    try {
      const double p_param = pid_gains_.at(i).at("p");
      const double i_param = pid_gains_.at(i).at("i");
      const double d_param = pid_gains_.at(i).at("d");
      const double i_max_param = pid_gains_.at(i).at("i_max");
      const double i_min_param = pid_gains_.at(i).at("i_min");

      pid.initPid(p_param, i_param, d_param, i_max_param, i_min_param, /*antiwindup*/ false);

      pids_.emplace_back(std::make_shared<control_toolbox::PidROS>(std::move(pid)));
    } catch (std::out_of_range &) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Did not find a complete pid configutation in the urdf for \"%s\"",
        this->virtual_joint_names_.at(i).c_str());
      pids_.emplace_back(nullptr);
    }
  }

  // listen to the update event (broadcast every simulation iteration)
  this->update_connection_ =
    event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GazeboPalHey5::UpdateChild, this));

  std::string init_str = "Initialized GazeboPalHey5 finger with actuator: " +
    this->actuated_joint_name_;
  init_str += " and virtual joints: ";
  for (unsigned int i = 0; i < virtual_joint_names_.size(); ++i) {
    init_str += this->virtual_joint_names_.at(i);
    init_str += " ";
  }
  RCLCPP_INFO_STREAM(ros_node_->get_logger(), init_str);
}

// Update the controller
void GazeboPalHey5::UpdateChild()
{
  ignition::math::Angle new_actuator_angle = actuated_joint_->Position(0u);

  // Filter for noisy measure of actuated angle
  const double ang_err_rad = (actuator_angle_ - new_actuator_angle).Radian();
  const double eps_angle_rad = 0.02;
  if (fabs(ang_err_rad) > eps_angle_rad) {
    actuator_angle_ = new_actuator_angle;
  }

  for (unsigned int i = 0; i < virtual_joints_.size(); ++i) {
    ignition::math::Angle new_angle = actuator_angle_ / scale_factors_.at(i);

    if (new_angle > virtual_joints_.at(i)->UpperLimit(0u)) {
      new_angle = virtual_joints_.at(i)->UpperLimit(0u);
    }
    if (new_angle < virtual_joints_.at(i)->LowerLimit(0u)) {
      new_angle = virtual_joints_.at(i)->LowerLimit(0u);
    }

    if (pids_.at(i)) {
      const double pos = virtual_joints_.at(i)->Position(0);
      const double error = new_angle.Radian() - pos;
      const double effort = pids_.at(i)->computeCommand(
        error,
        rclcpp::Duration::from_seconds(0.001));
      virtual_joints_.at(i)->SetForce(0, effort);
    } else {
      virtual_joints_.at(i)->SetPosition(0u, new_angle.Radian());
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboPalHey5)

}  // namespace gazebo
