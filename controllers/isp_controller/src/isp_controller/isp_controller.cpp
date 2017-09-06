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

ISP_Controller::ISP_Controller(const Params& params) : p_(params) {}

ControlCommand ISP_Controller::SD_Control(const cv::Mat& ISP,
                                          const ControlCommand& u_d) {
  // Reserve return value.
  ControlCommand cmd;

  // Map desired yaw image plane column.
  const auto col_d =
      yaw2Column(ISP, u_d.yaw, p_.focal_length_x, p_.principal_point_x);

  // Get control horizon.
  const auto horizon_row = static_cast<int>(ISP.rows * p_.kernel_horizon);
  const cv::Mat h = controlHorizon(ISP, p_.kernel_height, horizon_row);

  // Apply max filter.
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

  // Get safe controls from filtered horizon.
  const auto C_u =
      PotentialTransform<ConstraintType::SOFT>(p_.shape_parameters);
  const cv::Mat controls = safeControls(biased_h, C_u, p_.K_P, p_.K_D);

  // Get channel with throttle max values.
  std::vector<cv::Mat> control_channels(2);
  cv::split(controls, control_channels);

  // Find minimum.
  auto min_val = NaN;
  auto max_val = NaN;
  std::array<int, 2> min_idx;
  std::array<int, 2> max_idx;
  cv::minMaxIdx(control_channels[1], &min_val, &max_val, min_idx.data(),
                max_idx.data());

  // If maximum does not exceed inertia, revert to bias column.
  const auto p_bias_column_val = biased_h.at<cv::Point2d>(col_d).x;
  if (std::abs(p_bias_column_val - max_val) <= p_.potential_inertia) {
    max_idx[1] = col_d;
  }

  // Compute control command.
  const auto yaw_col_offset =
      static_cast<int>(static_cast<double>(max_idx[1]) - p_.principal_point_x);
  const auto yaw_star = column2Yaw(controls, yaw_col_offset, p_.focal_length_x,
                                   p_.principal_point_x);
  const cv::Point2d throttle_set = controls.at<cv::Point2d>(max_idx[1]);
  const auto throttle_star = nearestIntervalPoint(throttle_set, u_d.throttle);

  // Done.
  cmd.yaw = yaw_star;
  cmd.throttle = throttle_star;
  return cmd;
}
}  // namespace maeve_automation_core
