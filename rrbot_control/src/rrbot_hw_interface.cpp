/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the RRBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <rrbot_control/rrbot_hw_interface.h>

namespace rrbot_control {

RRBotHWInterface::RRBotHWInterface(ros::NodeHandle &nh,
                                   FlexRayHardwareInterface &&flex,
                                   urdf::Model *urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model),
      flex_{std::move(flex)} {
  ROS_INFO_NAMED("rrbot_hw_interface", "RRBotHWInterface Ready.");
  // joint_angle_pub_ =
  //  new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(
  //    nh, "joints", 4);
}

void RRBotHWInterface::read(ros::Duration &elapsed_time) {
  for (unsigned int i = 0; i < joint_names_.size(); i++) {
    flex_.read_muscle(joint_names_[i])
        .match(
            [&](muscleState_t &state) {
              joint_position_.at(i) = state.actuatorPos;
              joint_velocity_.at(i) = state.actuatorVel;
              joint_effort_.at(i) = state.tendonDisplacement;
              ROS_INFO_STREAM("joint angles for joint to be printed: "
                              << i << " is: " << state.jointPos);
            },
            [](FlexRayHardwareInterface::ReadError err) {
              switch (err) {
              case FlexRayHardwareInterface::ReadError::MuscleDoesNotExist:
                ROS_ERROR_STREAM(
                    "RRBotHWInterface: Error while reading muscle state: "
                    "MuscleDoesNotExist");
                break;

              case FlexRayHardwareInterface::ReadError::GanglionNotAttached:
                ROS_ERROR_STREAM("RRBotHWInterface: Error while "
                                 "reading muscle state: GanglionNotAttached");
                break;
              }
            });
  }
}

void RRBotHWInterface::write(ros::Duration &elapsed_time) {
  // Safety Not Guaranteed
  enforceLimits(elapsed_time);

  std::set<std::string> jointsWithPosCtrl =
      position_joint_interface_.getClaims();
  for (auto &&claims : jointsWithPosCtrl) {
    ROS_INFO_STREAM("position claims: " << claims);
  }
  std::set<std::string> jointsWithVelCtrl =
      velocity_joint_interface_.getClaims();
  for (auto &&claims : jointsWithVelCtrl) {
    ROS_INFO_STREAM("Velocity claims: " << claims);
  }
  std::set<std::string> jointsWithEffCtrl = effort_joint_interface_.getClaims();
  for (auto &&claims : jointsWithEffCtrl) {
    ROS_INFO_STREAM("Force claims: " << claims);
  }

  std::set<std::string> Pos = jointsWithPosCtrl;
  for (auto &&joint : jointsWithVelCtrl) {
    if (Pos.erase(joint)) {
      ROS_ERROR_STREAM(
          "RRBotHWInterface: Joint: "
          << joint << " is being claimed by more than one active controller.");
    }
  }
  //
  for (auto &&joint : jointsWithEffCtrl) {
    if (Pos.erase(joint)) {
      ROS_ERROR_STREAM(
          "RRBotHWInterface: Joint: "
          << joint << " is being claimed by more than one active controller.");
    }
  }

  std::set<std::string> Vel = jointsWithVelCtrl;
  for (auto &&joint : jointsWithPosCtrl) {
    if (Vel.erase(joint)) {
      ROS_ERROR_STREAM(
          "RRBotHWInterface: Joint: "
          << joint << " is being claimed by more than one active controller.");
    }
  }

  for (auto &&joint : jointsWithEffCtrl) {
    if (Vel.erase(joint)) {
      ROS_ERROR_STREAM(
          "RRBotHWInterface: Joint: "
          << joint << " is being claimed by more than one active controller.");
    }
  }

  std::set<std::string> Eff = jointsWithEffCtrl;
  for (auto &&joint : jointsWithPosCtrl) {
    if (Eff.erase(joint)) {
      ROS_ERROR_STREAM(
          "RRBotHWInterface: Joint: "
          << joint << " is being claimed by more than one active controller.");
    }
  }
  //
  for (auto &&joint : jointsWithVelCtrl) {
    if (Eff.erase(joint)) {
      ROS_ERROR_STREAM(
          "RRBotHWInterface: Joint: "
          << joint << " is being claimed by more than one active controller.");
    }
  }
  for (unsigned int i = 0; i < joint_names_.size(); ++i) {
    auto const &joint_name = joint_names_[i];

    if (Pos.count(joint_name)) {
      ROS_INFO_STREAM("Writing position command");
      flex_.set(joint_name, ControlMode::Position,
                joint_position_command_.at(i));
    }
    if (Vel.count(joint_name)) {
      ROS_INFO_STREAM("Writing velocity command");

      flex_.set(joint_name, ControlMode::Velocity,
                joint_velocity_command_.at(i));
    }
    if (Eff.count(joint_name)) {
      ROS_INFO_STREAM("Writing effort command");

      flex_.set(joint_name, ControlMode::Force, joint_effort_command_.at(i));
    }
    flex_.set(joint_name, ControlMode::Force, joint_effort_command_.at(i));
  }
}

void RRBotHWInterface::enforceLimits(ros::Duration &period) {
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

} // namespace
