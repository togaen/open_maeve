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
#include "maeve_automation_core/maeve_macros/checks.h"

namespace maeve_automation_core {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

std::ostream& operator<<(std::ostream& o,
                         const ISP_Controller::Params::ErosionKernel& ek) {
  return o << "[width: " << ek.width << ", height: " << ek.height
           << ", horizon: " << ek.horizon << "]";
}
std::ostream& operator<<(std::ostream& o,
                         const ISP_Controller::Params::HorizonDecay& hd) {
  return o << "[left: " << hd.left << ", right: " << hd.right << "]";
}
std::ostream& operator<<(std::ostream& o,
                         const ISP_Controller::Params::GuidanceGains& gg) {
  return o << "[throttle: " << gg.throttle << ", yaw: " << gg.yaw
           << ", control_set: " << gg.control_set << "]";
}

std::ostream& operator<<(std::ostream& o, const ISP_Controller::Params& p) {
  o << "focal_length_x: " << p.focal_length_x << "\n";
  o << "principal_point_x: " << p.principal_point_x << "\n";
  o << "K_P: " << p.K_P << "\n";
  o << "K_D: " << p.K_D << "\n";
  o << "potential_inertia: " << p.potential_inertia << "\n";
  o << "shape_parameters: " << p.shape_parameters << "\n";
  o << "erosion_kernel: " << p.erosion_kernel << "\n";
  o << "yaw_decay: " << p.yaw_decay << "\n";
  o << "guidance_gains: " << p.guidance_gains;
  return o;
}

ISP_Controller::Params::GuidanceGains::GuidanceGains()
    : GuidanceGains(NaN, NaN, NaN) {}

ISP_Controller::Params::GuidanceGains::GuidanceGains(const double t,
                                                     const double y,
                                                     const double c)
    : throttle(t), yaw(y), control_set(c) {}

bool ISP_Controller::Params::GuidanceGains::valid() const {
  // Perform checks.
  CHECK_STRICTLY_POSITIVE(throttle);
  CHECK_STRICTLY_POSITIVE(yaw);
  CHECK_STRICTLY_POSITIVE(control_set);
  CHECK_FINITE(throttle);
  CHECK_FINITE(yaw);
  CHECK_FINITE(control_set);

  // All passed.
  return true;
}

ISP_Controller::Params::HorizonDecay::HorizonDecay() : HorizonDecay(NaN, NaN) {}

ISP_Controller::Params::HorizonDecay::HorizonDecay(const double l,
                                                   const double r)
    : left(l), right(r) {}

bool ISP_Controller::Params::HorizonDecay::valid() const {
  // Perform checks.
  CHECK_CONTAINS_CLOSED(left, 0.0, 1.0);
  CHECK_CONTAINS_CLOSED(right, 0.0, 1.0);

  // All passed.
  return true;
}

ISP_Controller::Params::ErosionKernel::ErosionKernel()
    : ErosionKernel(NaN, NaN, NaN) {}

ISP_Controller::Params::ErosionKernel::ErosionKernel(const double w,
                                                     const double ht,
                                                     const double hr)
    : width(w), height(ht), horizon(hr) {}

bool ISP_Controller::Params::ErosionKernel::valid() const {
  // Perform checks.
  CHECK_CONTAINS_CLOSED(width, 0.0, 1.0);
  CHECK_CONTAINS_CLOSED(height, 0.0, 1.0);
  CHECK_CONTAINS_CLOSED(horizon, 0.0, 1.0);

  // All passed.
  return true;
}

ISP_Controller::Params::Params()
    : Params(ShapeParameters(), ErosionKernel(), HorizonDecay(),
             GuidanceGains(), NaN, NaN, NaN, NaN, NaN) {}

ISP_Controller::Params::Params(const ShapeParameters& sp,
                               const ErosionKernel& ek, const HorizonDecay& yd,
                               const GuidanceGains& gg, const double fx,
                               const double px, const double kp,
                               const double kd, const double pi)
    : focal_length_x(fx),
      principal_point_x(px),
      K_P(kp),
      K_D(kd),
      potential_inertia(pi),
      erosion_kernel(ek),
      yaw_decay(yd),
      guidance_gains(gg),
      shape_parameters(sp) {}

bool ISP_Controller::Params::valid() const {
  // Perform checks.
  CHECK_NOT_NAN(focal_length_x);
  CHECK_NOT_NAN(principal_point_x);
  CHECK_NOT_NAN(K_P);
  CHECK_NOT_NAN(K_D);
  CHECK_NOT_NAN(potential_inertia);
  return erosion_kernel.valid() && yaw_decay.valid() &&
         guidance_gains.valid() && shape_parameters.valid();
}

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
  const cv::Mat h =
      controlHorizon(ISP, p_.erosion_kernel.height, p_.erosion_kernel.horizon);

  // Apply min filter.
  const cv::Mat eroded_h = erodeHorizon(h, p_.erosion_kernel.width);

  // Compute guidance fields.
  const cv::Mat throttle_guidance = throttleGuidance(u_d.throttle, h.cols);
  const cv::Mat control_set_guidance = controlSetGuidance(throttle_guidance);
  const cv::Mat yaw_guidance =
      yawGuidance(col_d, h.cols, p_.yaw_decay.left, p_.yaw_decay.right);

  // Apply guidance fields.
  const cv::Mat guided_h =
      eroded_h + throttle_guidance + yaw_guidance + control_set_guidance;

  // Project throttles onto [r_min, r_max].
  const cv::Mat throttle_h =
      projectThrottlesToControlSpace(guided_h, C_u_, p_.K_P, p_.K_D);

  // Find the index of the desired control command.
  const auto control_idx =
      dampedMaxThrottleIndex(throttle_h, guided_h, p_.potential_inertia, col_d);

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
