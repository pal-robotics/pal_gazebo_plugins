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

#ifndef PAL_GAZEBO_PLUGINS__GAZEBO_UNDERACTUATED_FINGER_HPP_
#define PAL_GAZEBO_PLUGINS__GAZEBO_UNDERACTUATED_FINGER_HPP_

// std C++
#include <map>
#include <memory>
#include <string>
#include <vector>

// Gazebo
#include "control_toolbox/pid_ros.hpp"
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"

#include "gazebo_ros/node.hpp"
#include "rclcpp/rclcpp.hpp"


namespace gazebo
{

class Joint;
class Entity;

class GazeboPalHey5 : public ModelPlugin
{
public:
  GazeboPalHey5();
  ~GazeboPalHey5();
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  virtual void UpdateChild();

private:
  physics::WorldPtr world;
  physics::ModelPtr parent;
  event::ConnectionPtr update_connection_;

  std::string actuated_joint_name_;

  std::vector<std::string> virtual_joint_names_;

  physics::JointPtr actuated_joint_;

  std::vector<physics::JointPtr> virtual_joints_;

  using PidROSPtr = std::shared_ptr<control_toolbox::PidROS>;
  std::vector<PidROSPtr> pids_;
  std::vector<std::map<std::string, double>> pid_gains_;

  std::vector<double> scale_factors_;

  ignition::math::Angle actuator_angle_;

  std::string robot_namespace_;

  gazebo_ros::Node::SharedPtr ros_node_;
};

}  // namespace gazebo

#endif  // PAL_GAZEBO_PLUGINS__GAZEBO_UNDERACTUATED_FINGER_HPP_
