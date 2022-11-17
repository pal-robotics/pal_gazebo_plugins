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

#ifndef PAL_GAZEBO_PLUGINS__PAL_GAZEBO_COLLISIONS_HPP_
#define PAL_GAZEBO_PLUGINS__PAL_GAZEBO_COLLISIONS_HPP_

#include "rclcpp/rclcpp.hpp"

// Gazebo
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "gazebo_msgs/msg/contact_state.hpp"
#include "gazebo_ros/node.hpp"

namespace gazebo
{

class GazeboCollisions : public WorldPlugin
{
public:
  virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo &);

private:
  physics::WorldPtr world_;
  physics::Contact * contact_;

  gazebo::event::ConnectionPtr connection_;
  double update_rate_;
  rclcpp::Time last_time_published_;

  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<gazebo_msgs::msg::ContactsState>::SharedPtr collisions_pub_;
};

}  // namespace gazebo

#endif  // PAL_GAZEBO_PLUGINS__PAL_GAZEBO_COLLISIONS_HPP_
