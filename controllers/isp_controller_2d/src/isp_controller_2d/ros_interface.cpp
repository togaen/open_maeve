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

#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <limits>

#include "maeve_automation_core/isp_field/ros_interface.h"
#include "maeve_automation_core/isp_field/visualize.h"
#include "maeve_automation_core/ros_parameter_loading/macros.h"

namespace maeve_automation_core {
namespace {
const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

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
  const auto ek_ns = ns + "/erosion_kernel";
  LOAD_NAMED_PARAM(ek_ns + "/width", ek.width);
  LOAD_NAMED_PARAM(ek_ns + "/height", ek.height);
  LOAD_NAMED_PARAM(ek_ns + "/horizon", ek.horizon);

  auto& yd = params.yaw_decay;
  const auto yd_ns = ns + "/yaw_decay";
  LOAD_NAMED_PARAM(yd_ns + "/left", yd.left);
  LOAD_NAMED_PARAM(yd_ns + "/right", yd.right);

  auto& gg = params.guidance_gains;
  const auto gg_ns = ns + "/guidance_gains";
  LOAD_NAMED_PARAM(gg_ns + "/yaw", gg.yaw);
  LOAD_NAMED_PARAM(gg_ns + "/control_set", gg.control_set);

  LOAD_NAMED_PARAM(ns + "/K_P", params.K_P);
  LOAD_NAMED_PARAM(ns + "/K_D", params.K_D);

  // This one is tricky because it can be string or double.
  std::string str_inertia;
  if (!nh.getParam("params/potential_inertia", str_inertia)) {
    // If string version of the param fails to load, try to load as a double.
    LOAD_NAMED_PARAM(ns + "/potential_inertia", params.potential_inertia);
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

HorizonVisualizer::Params::Params()
    : horizon_viz_height(-1),
      constraint_range_min(NaN),
      constraint_range_max(NaN) {}

HorizonVisualizer::Params::Params(const int viz_height, const double r_min,
                                  const double r_max)
    : horizon_viz_height(viz_height),
      constraint_range_min(r_min),
      constraint_range_max(r_max) {}

bool HorizonVisualizer::Params::valid() const {
  return (horizon_viz_height > 0) && std::isfinite(constraint_range_min) &&
         std::isfinite(constraint_range_max);
}

void HorizonVisualizer::visualize(const std_msgs::Header& header,
                                  const ISP_Controller2D& controller) const {
  for (const auto& p : viz_horizon_pubs_) {
    const auto ht = ISP_Controller2D::stringToHorizonType(p.first);
    visualizeHorizon(header, controller.inspectHorizon(ht), ht, p.second);
  }
}

void HorizonVisualizer::initialize(const Params& params,
                                   const std::vector<std::string>& horizons,
                                   image_transport::ImageTransport& it) {
  params_ = params;
  std::for_each(
      std::begin(horizons), std::end(horizons), [&](const std::string& str) {
        viz_horizon_pubs_[str] = it.advertise("viz_" + str + "_horizon", 1);
      });
}

void HorizonVisualizer::visualizeHorizon(
    const std_msgs::Header& header, const cv::Mat& horizon,
    const ISP_Controller2D::HorizonType ht,
    const image_transport::Publisher& publisher) const {
  // Only proceed if parameters are valid.
  if (!params_.valid()) {
    return;
  }

  // Set visualization bounds according to horizon type.
  auto lower_bound = params_.constraint_range_min;
  auto upper_bound = params_.constraint_range_max;
  if ((ht == ISP_Controller2D::HorizonType::YAW_GUIDANCE) ||
      (ht == ISP_Controller2D::HorizonType::GUIDANCE) ||
      (ht == ISP_Controller2D::HorizonType::GUIDED_THROTTLE) ||
      (ht == ISP_Controller2D::HorizonType::CONTROL_SET_GUIDANCE)) {
    lower_bound = 0.0;
    upper_bound = 1.0;
  }

  // Set visualization channel according to horizon type.
  const auto channel =
      static_cast<int>(ht == ISP_Controller2D::HorizonType::GUIDED_THROTTLE);

  // Compute visualization message.
  const auto viz_horizon = computeHorizonVisualization(
      horizon, channel, params_.horizon_viz_height, params_.horizon_viz_height,
      lower_bound, upper_bound);
  sensor_msgs::ImagePtr viz_horizon_msg =
      cv_bridge::CvImage(header, "mono8", viz_horizon).toImageMsg();
  publisher.publish(viz_horizon_msg);
}
}  // namespace maeve_automation_core
