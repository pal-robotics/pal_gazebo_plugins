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

#include "physics/physics.hh"
#include "transport/transport.hh"
#include "gazebo_ros_diffdrive.h"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(PalDiffDrivePlugin)

enum {RIGHT, LEFT};

/////////////////////////////////////////////////
PalDiffDrivePlugin::PalDiffDrivePlugin()
{
  this->wheelSpeed[LEFT] = this->wheelSpeed[RIGHT] = 0;
}

/////////////////////////////////////////////////
void PalDiffDrivePlugin::Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->velSub = this->node->Subscribe(std::string("~/") +
                                       this->model->GetName() + "/vel_cmd", &PalDiffDrivePlugin::OnVelMsg, this);

  if (!_sdf->HasElement("left_joint"))
    gzerr << "DiffDrive plugin missing <left_joint> element\n";

  if (!_sdf->HasElement("right_joint"))
    gzerr << "DiffDrive plugin missing <right_joint> element\n";

  this->leftJoint = _model->GetJoint(
        _sdf->GetElement("left_joint")->GetValueString());
  this->rightJoint = _model->GetJoint(
        _sdf->GetElement("right_joint")->GetValueString());

  if (_sdf->HasElement("torque"))
    this->torque = _sdf->GetElement("torque")->GetValueDouble();
  else
  {
    gzwarn << "No torque value set for the DiffDrive plugin.\n";
    this->torque = 5.0;
  }

  if (!this->leftJoint)
    gzerr << "Unable to find left joint["
          << _sdf->GetElement("left_joint")->GetValueString() << "]\n";
  if (!this->rightJoint)
    gzerr << "Unable to find right joint["
          << _sdf->GetElement("right_joint")->GetValueString() << "]\n";

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PalDiffDrivePlugin::OnUpdate, this));

  // Initialize the ROS node and subscribe to cmd_vel
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "diff_drive_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(this->model->GetName());

  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  sub_ = rosnode_->subscribe("/cmd_vel",1, &PalDiffDrivePlugin::cmdVelCallback, this);
  pub_ = rosnode_->advertise<nav_msgs::Odometry>("/base_odometry/odom", 1);

  nav_msgs::Odometry odom;

  // Initialize the controller
  // Reset odometric pose
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;
}

/////////////////////////////////////////////////
void PalDiffDrivePlugin::Init()
{
  this->wheelSeparation = this->leftJoint->GetAnchor(0).Distance(
        this->rightJoint->GetAnchor(0));

  physics::EntityPtr parent = boost::shared_dynamic_cast<physics::Entity>(
        this->leftJoint->GetChild());

  math::Box bb = parent->GetBoundingBox();
  // This assumes that the largest dimension of the wheel is the diameter
  this->wheelRadius = bb.GetSize().GetMax() * 0.5;
}

/////////////////////////////////////////////////
void PalDiffDrivePlugin::OnVelMsg(ConstPosePtr &_msg)
{
  double vr, va;

  vr = _msg->position().x();
  va =  msgs::Convert(_msg->orientation()).GetAsEuler().z;

  this->wheelSpeed[LEFT] = vr + va * this->wheelSeparation / 2.0;
  this->wheelSpeed[RIGHT] = vr - va * this->wheelSeparation / 2.0;
}

void PalDiffDrivePlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  //    lock.lock();

  double vr, va;

  vr = cmd_msg->linear.x;
  va = cmd_msg->angular.z;


  this->wheelSpeed[RIGHT] = vr + va * this->wheelSeparation / 2.0;
  this->wheelSpeed[LEFT] = vr - va * this->wheelSeparation / 2.0;

}


/////////////////////////////////////////////////
void PalDiffDrivePlugin::OnUpdate()
{

  double leftVelDesired = (this->wheelSpeed[LEFT] / this->wheelRadius);
  double rightVelDesired = (this->wheelSpeed[RIGHT] / this->wheelRadius);

  this->leftJoint->SetVelocity(0, leftVelDesired);
  this->rightJoint->SetVelocity(0, rightVelDesired);

  this->leftJoint->SetMaxForce(0, this->torque);
  this->rightJoint->SetMaxForce(0, this->torque);

  //Odometry frames
  std::string odom_frame = tf::resolve(tf_prefix_, "odom");
  std::string base_footprint_frame = tf::resolve(tf_prefix_, "base_footprint");

  //Robot pose
  math::Pose pose = this->model->GetWorldPose();
  btQuaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
  btVector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

  //Publish transformation from odom to base
  ros::Time current_time = ros::Time::now();
  tf::Transform base_footprint_to_odom(qt, vt);
  transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                             current_time,
                                                             odom_frame,
                                                             base_footprint_frame));
  //Fill odom msg with:
  //Pose
  odom_.pose.pose.position.x = pose.pos.x;
  odom_.pose.pose.position.y = pose.pos.y;
  odom_.pose.pose.orientation.x = pose.rot.x;
  odom_.pose.pose.orientation.y = pose.rot.y;
  odom_.pose.pose.orientation.z = pose.rot.z;
  odom_.pose.pose.orientation.w = pose.rot.w;
  //Velocities
  math::Vector3 linear = this->model->GetWorldLinearVel();
  odom_.twist.twist.linear.x = linear.x;
  odom_.twist.twist.linear.y = linear.y;
  odom_.twist.twist.angular.z = this->model->GetWorldAngularVel().z;
  //Covariance
  odom_.twist.covariance[0] = 1e-3; odom_.twist.covariance[7] = 1e-3; odom_.twist.covariance[14] = 1e6;
  odom_.twist.covariance[21] = 1e6; odom_.twist.covariance[28] = 1e6; odom_.twist.covariance[35] = 1e3;
  odom_.pose.covariance = odom_.twist.covariance;
  //Header
  odom_.header.stamp = current_time;
  odom_.header.frame_id = odom_frame;
  odom_.child_frame_id = base_footprint_frame;

  //Publish
  pub_.publish(odom_);
}
