/*
 *  gazebo_pal_hand.h
 *  Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
 *  Created on: 7 Jan 2014
 *      Author: luca
   * \brief A plugin for gazebo for controlling the pal underactuated hand in simulation
   * \author  Luca Marchionni (luca.marchionni@pal-robotics.com)
 */

#ifndef GAZEBO_PAL_HAND_H
#define GAZEBO_PAL_HAND_H

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <control_toolbox/pid.h>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboPalHand : public ModelPlugin {

    public:
      GazeboPalHand();
      ~GazeboPalHand();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
      virtual void UpdateChild();

    private:

      physics::WorldPtr world;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      std::string finger_joint_name_;
      std::string finger_1_joint_name_;
      std::string finger_2_joint_name_;
      std::string finger_3_joint_name_;

      physics::JointPtr joints[4];

      std::string robot_namespace_;

  };

}

#endif // GAZEBO_PAL_HAND_H
