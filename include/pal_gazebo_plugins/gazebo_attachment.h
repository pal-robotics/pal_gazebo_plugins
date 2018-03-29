/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/
#ifndef GAZEBO_ATTACHMENT_H
#define GAZEBO_ATTACHMENT_H

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
/**
 * @brief The GazeboAttachment class creates a joint between a link of the model
 * that contains this plugin and another model's link
 *
 * Example of the parameters required:
 *         <target_model_name>tiago</target_model_name>
 *         <target_link_name>arm_6_link</target_link_name>
 *         <local_link_name>my_model_link</local_link_name>
 *         <pose>
 *           0.0 0.5 0.0 0.0 0.0 1.57
 *         </pose>
 *
 *  Pose is: x y z r p y
 */
class GazeboAttachment : public ModelPlugin
{
public:
  GazeboAttachment();
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  virtual void Init() override;

private:
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::ModelPtr target_model_;
  physics::LinkPtr target_link_;
  physics::LinkPtr local_link_;
  ignition::math::Pose3d pose_;
  bool load_ok_;
};
}


#endif  // GAZEBO_ATTACHMENT_H
