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

ISP_Controller::Params::Params()
    : kernel_width(-1),
      kernel_height(-1),
      kernel_horizon(-1),
      focal_length_x(NaN),
      principal_point_x(NaN),
      theta_decay_left(NaN),
      theta_decay_right(NaN),
      K_P(NaN),
      K_D(NaN),
      potential_inertia(NaN) {}

ISP_Controller::ControlCommand::ControlCommand() : throttle(NaN), yaw(NaN) {}

ISP_Controller::ControlCommand::ControlCommand(const double t, const double y)
    : throttle(t), yaw(y) {}

ISP_Controller::ISP_Controller(const Params& params,
                               const ControlCommand& initial_commanded_control)
    : p_(params), commanded_control_(initial_commanded_control) {}

ISP_Controller::ControlCommand ISP_Controller::SD_Control(
    const cv::Mat& ISP, const ControlCommand& u_d) {
  // Reserve return value.
  ControlCommand cmd;

  // Map desired yaw image plane column.
  const auto col_d =
      theta2Column(ISP, u_d.yaw, p_.focal_length_x, p_.principal_point_x);

  // Get control horizon.
  const cv::Mat h = controlHorizon(ISP, p_.kernel_height, p_.kernel_horizon);

  // Apply max filter.
  const cv::Mat dilated_h = dilateHorizon(h, p_.kernel_width);

  // Get safe controls from filtered horizon.
  const auto C_u =
      PotentialTransform<ConstraintType::SOFT>(p_.shape_parameters);
  const cv::Mat safe_controls = safeControls(dilated_h, C_u, p_.K_P, p_.K_D);

  // Compute biasing fields.
  const cv::Mat theta_biasing =
      thetaBias(col_d, h.cols, p_.theta_decay_left, p_.theta_decay_right);
  const cv::Mat accel_biasing = accelBias(safe_controls);

  // Apply biasing fields.
  const cv::Mat biased_h = dilated_h.mul(theta_biasing.mul(accel_biasing));

  // Find minimum.
  double min_val = NaN;
  double max_val = NaN;
  std::array<int, 2> min_idx;
  std::array<int, 2> max_idx;
  cv::minMaxIdx(biased_h.reshape(1), &min_val, &max_val, min_idx.data(),
                max_idx.data());

  // If minimum does not exceed inertia, revert to bias column.
  const auto p_bias_column = biased_h.at<cv::Point2d>(col_d).x;
  if (std::abs(p_bias_column - min_val) <= p_.potential_inertia) {
    min_idx[1] = col_d;
  }

  // Compute control command.
  const auto midpoint = ISP.cols / 2;
  const auto theta_offset = min_idx[1] - midpoint;
  const cv::Point2d accel_set = safe_controls.at<cv::Point2d>(min_idx[1]);
  // \TODO(me)

  // Done.
  return cmd;
}
}  // namespace maeve_automation_core
