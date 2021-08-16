/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef PAL_GAZEBO_PLUGINS__GAZEBO_WORLD_ODOMETRY_HPP_
#define PAL_GAZEBO_PLUGINS__GAZEBO_WORLD_ODOMETRY_HPP_

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>

namespace gazebo
{

class Entity;

class GazeboWorldOdometry : public ModelPlugin
{
public:
  GazeboWorldOdometry();
  ~GazeboWorldOdometry();
  void DeferredLoad();
  void Load(physics::ModelPtr parent_model, sdf::ElementPtr sdf);

protected:
  virtual void UpdateChild();

private:
  void RosQueueThread();

  /// \brief for setting ROS name space
  std::string robot_namespace_;
  std::string topic_name_;
  std::string frame_name_;
  std::string body_name_;

  rclcpp::Node::SharedPtr rosNode_;
  double update_rate_;
  common::Time last_time_;
  gazebo::physics::WorldPtr world_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr floatingBasePub_;
  gazebo::physics::LinkPtr link;

  event::ConnectionPtr update_connection_;
  std::thread deferredLoadThread;
  std::thread callbackQueeuThread;

  // Mutex
  std::mutex mutex_;

  double x_offset_;
  double y_offset_;
  double yaw_offset_;
  double gaussian_noise_;

  std::default_random_engine generator_;
  double noise_x_;
  double noise_y_;
  double noise_yaw_;
};

}  // namespace gazebo

#endif  // PAL_GAZEBO_PLUGINS__GAZEBO_WORLD_ODOMETRY_HPP_
