/*
 * Copyright 2021 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#ifndef GAZEBO_COLLISIONS_H
#define GAZEBO_COLLISIONS_H

#include <ros/ros.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ContactState.h>

namespace gazebo
{

class GazeboCollisions : public WorldPlugin
{
public:
  virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo &);

private:
  physics::WorldPtr world_;
  physics::Contact *contact_;

  gazebo::event::ConnectionPtr connection_;
  double update_rate_;
  ros::Time last_time_published_;

  ros::NodeHandle n_;
  ros::Publisher collisions_pub_;
};

}  // namespace

#endif  // GAZEBO_COLLISIONS_H
