/*
 * Copyright 2021 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <pal_gazebo_plugins/pal_gazebo_collisions.h>

namespace gazebo
{
void GazeboCollisions::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->last_time_published_ = ros::Time::now();
  this->world_ = _world;
  this->collisions_pub_ =
      this->n_.advertise<gazebo_msgs::ContactsState>("/pal_gazebo_contacts", 1000);
  this->connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboCollisions::OnUpdate, this, std::placeholders::_1));

  // Activate contacts computation
  this->world_->Physics()->GetContactManager()->SetNeverDropContacts(true);

  this->update_rate_ = 1000.0;
  if (!_sdf->HasElement("updateRate"))
  {
    ROS_INFO("world odometry plugin missing <updateRate>, defaults to %f", this->update_rate_);
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
}

void GazeboCollisions::OnUpdate(const common::UpdateInfo &)
{
  if ((ros::Time::now() - this->last_time_published_).toSec() >= 1 / this->update_rate_)
  {
    int n = this->world_->Physics()->GetContactManager()->GetContactCount();

    gazebo_msgs::ContactsState css;

    css.header.seq = 0;
    css.header.stamp = ros::Time::now();
    css.header.frame_id = "world";

    for (int i = 0; i < n; i++)
    {
      this->contact_ = this->world_->Physics()->GetContactManager()->GetContact(i);

      gazebo_msgs::ContactState cs;

      cs.collision1_name = this->contact_->collision1->GetScopedName();
      cs.collision2_name = this->contact_->collision2->GetScopedName();

      geometry_msgs::Vector3 normal_vector;
      for (ignition::math::Vector3d normal : this->contact_->normals)
      {
        normal_vector.x = normal.X();
        normal_vector.y = normal.Y();
        normal_vector.z = normal.Z();
        cs.contact_normals.push_back(normal_vector);
      }

      geometry_msgs::Vector3 position_vector;
      for (ignition::math::Vector3d position : this->contact_->positions)
      {
        position_vector.x = position.X();
        position_vector.y = position.Y();
        position_vector.z = position.Z();
        cs.contact_positions.push_back(position_vector);
      }

      cs.depths.insert(cs.depths.begin(), std::begin(this->contact_->depths),
                       std::end(this->contact_->depths));

      std::stringstream info;
      info << "collision between " << cs.collision1_name << " and " << cs.collision2_name;
      cs.info = info.str();

      cs.total_wrench.force.x =
          this->contact_->wrench->body1Force.X() + this->contact_->wrench->body2Force.X();
      cs.total_wrench.force.y =
          this->contact_->wrench->body1Force.Y() + this->contact_->wrench->body2Force.Y();
      cs.total_wrench.force.z =
          this->contact_->wrench->body1Force.Z() + this->contact_->wrench->body2Force.Z();
      cs.total_wrench.torque.x =
          this->contact_->wrench->body1Torque.X() + this->contact_->wrench->body2Torque.X();
      cs.total_wrench.torque.y =
          this->contact_->wrench->body1Torque.Y() + this->contact_->wrench->body2Torque.Y();
      cs.total_wrench.torque.z =
          this->contact_->wrench->body1Torque.Z() + this->contact_->wrench->body2Torque.Z();

      geometry_msgs::Wrench w1;
      w1.force.x = this->contact_->wrench->body1Force.X();
      w1.force.y = this->contact_->wrench->body1Force.Y();
      w1.force.z = this->contact_->wrench->body1Force.Z();
      w1.torque.x = this->contact_->wrench->body1Torque.X();
      w1.torque.y = this->contact_->wrench->body1Torque.Y();
      w1.torque.z = this->contact_->wrench->body1Torque.Z();
      cs.wrenches.push_back(w1);

      geometry_msgs::Wrench w2;
      w2.force.x = this->contact_->wrench->body2Force.X();
      w2.force.y = this->contact_->wrench->body2Force.Y();
      w2.force.z = this->contact_->wrench->body2Force.Z();
      w2.torque.x = this->contact_->wrench->body2Torque.X();
      w2.torque.y = this->contact_->wrench->body2Torque.Y();
      w2.torque.z = this->contact_->wrench->body2Torque.Z();
      cs.wrenches.push_back(w2);

      css.states.push_back(cs);
    }

    this->collisions_pub_.publish(css);
    this->last_time_published_ = ros::Time::now();
  }
}

GZ_REGISTER_WORLD_PLUGIN(GazeboCollisions)
}
