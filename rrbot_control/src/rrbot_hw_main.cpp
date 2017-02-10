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
   Desc:   Example ros_control main() entry point for controlling robots in ROS
*/

#include <flexrayusbinterface/Parsers.hpp>
#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <rrbot_control/rrbot_hw_interface.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "rrbot_hw_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Create the hardware interface specific to your robot
  std::string bridge_description;
  if (!nh.getParam("/flex_bridge", bridge_description)) {
    ROS_ERROR_STREAM(
        "Please provide an ftdi device in ros parameter /myo_blink/ftdi_id");
    return 1;
  }
  try {
    auto node = YAML::Load(bridge_description);
    ROS_INFO_STREAM("Description parsed");
    node = node["FlexRay"];
    ROS_INFO_STREAM("Fetched yaml data");

    FlexRayBus fbus = node.as<FlexRayBus>();
    while (FlexRayHardwareInterface::connect(std::move(fbus))
               .match(
                   [&](FlexRayHardwareInterface &flex) {
                     ROS_INFO_STREAM("Connected");
                     // todo: get URDF file to pass in to replace the NULL
                     if (nh.hasParam("hardware_interface/joints")) {
                       nh.deleteParam("hardware_interface/joints");
                     }
                     nh.setParam("hardware_interface/joints",
                                 flex.get_muscle_names());
                     boost::shared_ptr<rrbot_control::RRBotHWInterface>
                         rrbot_hw_interface(new rrbot_control::RRBotHWInterface(
                             nh, std::move(flex), NULL));
                     rrbot_hw_interface->init();

                     // Start the control loop
                     ros_control_boilerplate::GenericHWControlLoop control_loop(
                         nh, rrbot_hw_interface);
                     return false;
                   },
                   [&](std::pair<FlexRayBus, FtResult> &result) {
                     ROS_ERROR_STREAM("Could not connect to the myo motor: "
                                      << result.second.str());
                     fbus = std::move(result.first);
                     return true;
                   }))
      ;
  } catch (YAML::Exception e) {
    ROS_ERROR_STREAM("Error in /flex_bridge["
                     << e.mark.pos << "]:" << e.mark.line << ":"
                     << e.mark.column << ": " << e.msg);
  }

  // Wait until shutdown signal recieved
  ros::waitForShutdown();

  return 0;
}
