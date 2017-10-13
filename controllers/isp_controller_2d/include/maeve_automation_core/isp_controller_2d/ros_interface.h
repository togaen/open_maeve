/*
 * Copyright 2017 Maeve Automation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#pragma once

#include "controller_interface_msgs/Command2D.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"

#include "maeve_automation_core/isp_controller_2d/control_command.h"
#include "maeve_automation_core/isp_controller_2d/isp_controller_2d.h"

namespace maeve_automation_core {
/**
 * @brief Serialize and return this object.
 *
 * @return The serialized version of this object.
 */
controller_interface_msgs::Command2D controlCommand2Command2D_Msg(
    const ControlCommand& cmd, const std_msgs::Header& header);

/**
 * @brief Deserialize and return a control command object.
 *
 * @param msg The serialized version.
 *
 * @return The deserialized version.
 */
ControlCommand command2D_Msg2ControlCommand(
    const controller_interface_msgs::Command2D& msg);

/**
 * @brief Convenience method for filling a parameter object from the ROS
 * parameter server.
 *
 * @note Shape parameters for the ISP controller must live in a namespace
 * 'shape_parameters' that is relative to ns.
 *
 * @param nh ROS node handle.
 * @param ns The node-relative namespace that contains the controller params.
 * @param isp_controller_params The parameter object to fill out.
 *
 * @return True if 'isp_controller_params' was successfully filled out;
 * otherwise false.
 */
bool loadISP_ControllerROS_Params(const ros::NodeHandle& nh,
                                  const std::string& ns,
                                  ISP_Controller2D::Params& params);
}  // namespace maeve_automation_core
