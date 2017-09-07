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
#include "maeve_automation_core/isp_controller/isp_controller.h"

#include <array>
#include <limits>

#include "isp_controller/lib.h"

namespace maeve_automation_core {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

std::ostream& operator<<(std::ostream& o, const ISP_Controller::Params& p) {
  o << "kernel_width: " << p.kernel_width << "\n";
  o << "kernel_height: " << p.kernel_height << "\n";
  o << "kernel_horizon: " << p.kernel_horizon << "\n";
  o << "focal_length_x: " << p.focal_length_x << "\n";
  o << "principal_point_x: " << p.principal_point_x << "\n";
  o << "yaw_decay_left: " << p.yaw_decay_left << "\n";
  o << "yaw_decay_right: " << p.yaw_decay_right << "\n";
  o << "K_P: " << p.K_P << "\n";
  o << "K_D: " << p.K_D << "\n";
  o << "potential_inertia: " << p.potential_inertia << "\n";
  o << "shape_parameters: " << p.shape_parameters;
  return o;
}

ISP_Controller::Params::Params()
    : kernel_width(-1),
      kernel_height(-1),
      kernel_horizon(NaN),
      focal_length_x(NaN),
      principal_point_x(NaN),
      yaw_decay_left(NaN),
      yaw_decay_right(NaN),
      K_P(NaN),
      K_D(NaN),
      potential_inertia(NaN) {}

ISP_Controller::Params::Params(const ShapeParameters& sp, const int k_w,
                               const int k_ht, const double k_hr,
                               const double fx, const double px,
                               const double ld, const double rd,
                               const double kp, const double kd,
                               const double pi)
    : kernel_width(k_w),
      kernel_height(k_ht),
      kernel_horizon(k_hr),
      focal_length_x(fx),
      principal_point_x(px),
      yaw_decay_left(ld),
      yaw_decay_right(rd),
      K_P(kp),
      K_D(kd),
      potential_inertia(pi),
      shape_parameters(sp) {}

ISP_Controller::ISP_Controller(const Params& params)
    : p_(params), C_u_(p_.shape_parameters) {}

ControlCommand ISP_Controller::SD_Control(const cv::Mat& ISP,
                                          const ControlCommand& u_d) {
  // Reserve return value.
  ControlCommand cmd;

  // Map desired yaw image plane column.
  const auto col_d =
      yaw2Column(ISP, u_d.yaw, p_.focal_length_x, p_.principal_point_x);

  // Get control horizon.
  const cv::Mat h = controlHorizon(ISP, p_.kernel_height, p_.kernel_horizon);

  // Apply min filter.
  const cv::Mat eroded_h = erodeHorizon(h, p_.kernel_width);

  // Compute guidance fields.
  const cv::Mat throttle_guidance = throttleGuidance(u_d.throttle, h.cols);

  // Apply guidance fields.
  const cv::Mat guided_h = eroded_h + throttle_guidance;

  // Compute biasing fields.
  const cv::Mat yaw_biasing =
      yawBias(col_d, h.cols, p_.yaw_decay_left, p_.yaw_decay_right);
  const cv::Mat control_set_biasing = controlSetBias(guided_h);

  // Apply biasing fields.
  const cv::Mat biased_h = guided_h.mul(control_set_biasing.mul(yaw_biasing));

  // Project throttles onto [r_min, r_max].
  const cv::Mat throttle_h =
      projectThrottlesToControlSpace(biased_h, C_u_, p_.K_P, p_.K_D);

  // Find the index of the desired control command.
  const auto control_idx =
      dampedMaxThrottleIndex(throttle_h, biased_h, p_.potential_inertia, col_d);

  // Compute yaw control command.
  const auto yaw = column2Yaw(throttle_h, control_idx, p_.focal_length_x,
                              p_.principal_point_x);

  // Project yaw to [range_min, range_max].
  cmd.yaw = projectYawToControlSpace(throttle_h, C_u_, p_.focal_length_x,
                                     p_.principal_point_x, yaw);

  // Compute throttle control command (it is already projected by C_u_).
  const cv::Point2d throttle_set = throttle_h.at<cv::Point2d>(control_idx);
  cmd.throttle =
      projectToInterval(throttle_set.x, throttle_set.y, u_d.throttle);

  // Done.
  return cmd;
}
}  // namespace maeve_automation_core
