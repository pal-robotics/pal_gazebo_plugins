/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "common/common.hh"
#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "gazebo.hh"

namespace gazebo
{
  class PalDiffDrivePlugin : public ModelPlugin
  {
  public: PalDiffDrivePlugin();
  public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  public: virtual void Init();

  private: void OnUpdate();
  private: void Update();

  private: void OnVelMsg(ConstPosePtr &_msg);

    // DiffDrive stuff
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
  void publishOdometry();

    // ROS STUFF
    ros::NodeHandle* rosnode_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    tf::TransformBroadcaster *transform_broadcaster_;
    nav_msgs::Odometry odom_;
    std::string tf_prefix_;

  private: transport::NodePtr node;
  private: transport::SubscriberPtr velSub;

  private: physics::ModelPtr model;
  private: physics::JointPtr leftJoint, rightJoint;
  private: event::ConnectionPtr updateConnection;
  private: double wheelSpeed[2];
  private: double torque;
  private: double wheelSeparation;
  private: double wheelRadius;

  private: physics::LinkPtr link, leftWheelLink, rightWheelLink;

  private: double sum;
    double x_;
    double rot_;
    bool alive_;
    double odomPose[3],odomVel[3];
  };
}

