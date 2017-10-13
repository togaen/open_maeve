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
#include "maeve_automation_core/isp_controller_2d/ros_interface.h"

#include "maeve_automation_core/isp_field/ros_interface.h"
#include "maeve_automation_core/ros_parameter_loading/macros.h"

namespace maeve_automation_core {
controller_interface_msgs::Command2D controlCommand2Command2D_Msg(
    const ControlCommand& cmd, const std_msgs::Header& header) {
  controller_interface_msgs::Command2D msg;
  msg.header = header;
  msg.x = cmd.throttle;
  msg.y = cmd.yaw;
  return msg;
}

ControlCommand command2D_Msg2ControlCommand(
    const controller_interface_msgs::Command2D& msg) {
  return ControlCommand(msg.x, msg.y);
}

bool loadISP_ControllerROS_Params(const ros::NodeHandle& nh,
                                  const std::string& ns,
                                  ISP_Controller2D::Params& params) {
  // Load shape parameters for controller.
  if (!loadShapeParamsROS_Params(nh, ns + "/shape_parameters",
                                 params.shape_parameters)) {
    return false;
  }

  // Load controller parameters.
  auto& ek = params.erosion_kernel;
  LOAD_NS_NAME_PARAM(ns, ek.width);
  LOAD_NS_NAME_PARAM(ns, ek.height);
  LOAD_NS_NAME_PARAM(ns, ek.horizon);

  auto& yd = params.yaw_decay;
  LOAD_NS_NAME_PARAM(ns, yd.left);
  LOAD_NS_NAME_PARAM(ns, yd.right);

  auto& gg = params.guidance_gains;
  LOAD_NS_NAME_PARAM(ns, gg.throttle);
  LOAD_NS_NAME_PARAM(ns, gg.yaw);
  LOAD_NS_NAME_PARAM(ns, gg.control_set);

  LOAD_NS_NAME_PARAM(ns, params.K_P);
  LOAD_NS_NAME_PARAM(ns, params.K_D);

  // This one is tricky because it can be string or double.
  std::string str_inertia;
  if (!nh.getParam("params/potential_inertia", str_inertia)) {
    // If string version of the param fails to load, try to load as a double.
    LOAD_NS_NAME_PARAM(ns, params.potential_inertia);
  } else {
    char* pEnd = nullptr;
    params.potential_inertia = std::strtod(str_inertia.c_str(), &pEnd);
    if (*pEnd != '\0') {
      // The full string was not consumed by strtod. This is invalid input.
      ROS_FATAL_STREAM(
          "Failed processing potential inertia parameter: " << pEnd);
      return false;
    }
  }

  return true;
}
}  // namespace maeve_automation_core
