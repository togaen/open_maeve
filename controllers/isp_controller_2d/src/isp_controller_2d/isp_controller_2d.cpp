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
#include "open_maeve/isp_controller_2d/isp_controller_2d.h"

#include <array>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "isp_controller_2d/lib.h"
#include "open_maeve/maeve_geometry/interval.h"
#include "open_maeve/maeve_macros/checks.h"

namespace open_maeve {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

//------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& o,
                         const ISP_Controller2D::Params::ErosionKernel& ek) {
  return o << "[width: " << ek.width << ", height: " << ek.height
           << ", horizon: " << ek.horizon << "]";
}

//------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& o,
                         const ISP_Controller2D::Params::HorizonDecay& hd) {
  return o << "[left: " << hd.left << ", right: " << hd.right << "]";
}

//------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& o,
                         const ISP_Controller2D::Params::GuidanceGains& gg) {
  return o << "[yaw: " << gg.yaw << ", control_set: " << gg.control_set << "]";
}

//------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& o, const ISP_Controller2D::Params& p) {
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

//------------------------------------------------------------------------------

ISP_Controller2D::Params::GuidanceGains::GuidanceGains()
    : GuidanceGains(NaN, NaN) {}

//------------------------------------------------------------------------------

ISP_Controller2D::Params::GuidanceGains::GuidanceGains(const double y,
                                                       const double c)
    : yaw(y), control_set(c) {}

//------------------------------------------------------------------------------

bool ISP_Controller2D::Params::GuidanceGains::valid() const {
  // Perform checks.
  CHECK_GE(yaw, 0.0);
  CHECK_GE(control_set, 0.0);
  CHECK_FINITE(yaw);
  CHECK_FINITE(control_set);

  // All passed.
  return true;
}

//------------------------------------------------------------------------------

ISP_Controller2D::Params::HorizonDecay::HorizonDecay()
    : HorizonDecay(NaN, NaN) {}

//------------------------------------------------------------------------------

ISP_Controller2D::Params::HorizonDecay::HorizonDecay(const double l,
                                                     const double r)
    : left(l), right(r) {}

//------------------------------------------------------------------------------

bool ISP_Controller2D::Params::HorizonDecay::valid() const {
  // Perform checks.
  CHECK_CONTAINS_CLOSED(left, 0.0, 1.0);
  CHECK_CONTAINS_CLOSED(right, 0.0, 1.0);

  // All passed.
  return true;
}

//------------------------------------------------------------------------------

ISP_Controller2D::Params::ErosionKernel::ErosionKernel()
    : ErosionKernel(NaN, NaN, NaN) {}

//------------------------------------------------------------------------------

ISP_Controller2D::Params::ErosionKernel::ErosionKernel(const double w,
                                                       const double ht,
                                                       const double hr)
    : width(w), height(ht), horizon(hr) {}

//------------------------------------------------------------------------------

bool ISP_Controller2D::Params::ErosionKernel::valid() const {
  // Perform checks.
  CHECK_CONTAINS_CLOSED(width, 0.0, 1.0);
  CHECK_CONTAINS_CLOSED(height, 0.0, 1.0);
  CHECK_CONTAINS_CLOSED(horizon, 0.0, 1.0);

  // All passed.
  return true;
}

//------------------------------------------------------------------------------

ISP_Controller2D::Params::Params()
    : Params(ShapeParameters(), ErosionKernel(), HorizonDecay(),
             GuidanceGains(), NaN, NaN, NaN, NaN, NaN) {}

//------------------------------------------------------------------------------

ISP_Controller2D::Params::Params(const ShapeParameters& sp,
                                 const ErosionKernel& ek,
                                 const HorizonDecay& yd,
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

//------------------------------------------------------------------------------

bool ISP_Controller2D::Params::valid() const {
  // Perform checks.
  CHECK_GT(focal_length_x, 0.0);
  CHECK_NOT_NAN(principal_point_x);
  CHECK_NOT_NAN(K_P);
  CHECK_NOT_NAN(K_D);
  CHECK_NOT_NAN(potential_inertia);
  CHECK_GE(potential_inertia, 0.0);
  return erosion_kernel.valid() && yaw_decay.valid() &&
         guidance_gains.valid() && shape_parameters.valid();
}

//------------------------------------------------------------------------------

ISP_Controller2D::ISP_Controller2D() : init_(false) {}

//------------------------------------------------------------------------------

ISP_Controller2D::ISP_Controller2D(const Params& params)
    : init_(true), p_(params), C_u_(p_.shape_parameters) {}

//------------------------------------------------------------------------------

bool ISP_Controller2D::isInitialized() const { return init_ && p_.valid(); }

//------------------------------------------------------------------------------

const cv::Mat& ISP_Controller2D::inspectHorizon(const HorizonType cs) const {
  static const cv::Mat empty;

  // Get horizon structure.
  const auto it = horizons_.find(cs);

  // The find should never fail. If it does, implementation is inconsistent.
  if (it == std::end(horizons_)) {
    std::stringstream ss;
    ss << "Unable to find requested horizon structure: "
       << horizonTypeToString(cs) << ". This should not happen.";
    throw std::runtime_error(ss.str());
  }

  // Done.
  return it->second;
}

//------------------------------------------------------------------------------

std::string ISP_Controller2D::horizonTypeToString(const HorizonType cs) {
  switch (cs) {
    case HorizonType::CONTROL:
      return "control";
    case HorizonType::ERODED_CONTROL:
      return "eroded_control";
    case HorizonType::THROTTLE:
      return "throttle";
    case HorizonType::GUIDED_THROTTLE:
      return "guided_throttle";
    case HorizonType::YAW_GUIDANCE:
      return "yaw_guidance";
    default:
      return "invalid";
  }
}

//------------------------------------------------------------------------------

ISP_Controller2D::HorizonType ISP_Controller2D::stringToHorizonType(
    const std::string& str) {
  if (str == "control") {
    return HorizonType::CONTROL;
  }
  if (str == "eroded_control") {
    return HorizonType::ERODED_CONTROL;
  }
  if (str == "throttle") {
    return HorizonType::THROTTLE;
  }
  if (str == "guided_throttle") {
    return HorizonType::GUIDED_THROTTLE;
  }
  if (str == "yaw_guidance") {
    return HorizonType::YAW_GUIDANCE;
  }
  return HorizonType::INVALID;
}

//------------------------------------------------------------------------------

const ISP_Controller2D::Params& ISP_Controller2D::get_params(
    const ISP_Controller2D& c) {
  return c.p_;
}

//------------------------------------------------------------------------------

ControlCommand ISP_Controller2D::potentialControl(const cv::Mat& ISP,
                                                  const cv::Rect& ROI) {
  // Reserve return value.
  ControlCommand u_d;

  // Compute generic horizons.
  //  const cv::Rect ROI = control_horizon_ROI(ISP, p_.erosion_kernel.height,
  //                                           p_.erosion_kernel.horizon);
  computeControlSelectionHorizon(ISP, ROI);

  // Map desired yaw image plane column.
  const auto col_d =
      yaw2Column(ISP, 0.0, p_.focal_length_x, p_.principal_point_x);

  // Find the index of the desired control command.
  const auto throttle_h = horizons_[HorizonType::THROTTLE];
  const auto control_idx =
      dampedMaxThrottleIndex(throttle_h, p_.potential_inertia, col_d);

  // Compute yaw control command.
  const auto yaw =
      column2Yaw(throttle_h, static_cast<double>(control_idx) + 0.5,
                 p_.focal_length_x, p_.principal_point_x);

  // \TODO(CD-47) make separate constraint projection for yaw; for now, undo
  // translation.
  u_d.yaw = C_u_(cv::Point(p_.guidance_gains.yaw * yaw +
                               p_.shape_parameters.translation,
                           0.0))
                .x;

  // Compute throttle control command (it is already projected by C_u_).
  const cv::Point2d throttle_set = throttle_h.at<cv::Point2d>(control_idx);
  u_d.throttle = throttle_set.y;

  // Done.
  return u_d;
}

//------------------------------------------------------------------------------

void ISP_Controller2D::computeControlSelectionHorizon(const cv::Mat& ISP,
                                                      const cv::Rect& ROI) {
  // Get control horizon.
  horizons_[HorizonType::CONTROL] = avg_reduce_to_horizon(ISP, ROI);
  const auto ch = horizons_[HorizonType::CONTROL];

  // Apply filter.
  horizons_[HorizonType::ERODED_CONTROL] =
      erodeHorizon(ch, p_.erosion_kernel.width);
  const auto ech = horizons_[HorizonType::ERODED_CONTROL];

  // Project throttles onto [r_min, r_max].
  horizons_[HorizonType::THROTTLE] =
      projectThrottlesToControlSpace(ech, C_u_, p_.K_P, p_.K_D);
}

//------------------------------------------------------------------------------

ControlCommand ISP_Controller2D::rememberCommand(const ControlCommand& cmd) {
  last_computed_cmd_ = cmd;
  return last_computed_cmd_;
}

//------------------------------------------------------------------------------

ControlCommand ISP_Controller2D::SD_Control(const cv::Mat& ISP,
                                            const cv::Rect& ROI,
                                            const ControlCommand& u_d) {
  // Reserve return value.
  ControlCommand cmd;

  // Compute generic horizons.
  computeControlSelectionHorizon(ISP, ROI);

  // Map desired yaw image plane column.
  const auto col_d =
      yaw2Column(ISP, u_d.yaw, p_.focal_length_x, p_.principal_point_x);

  // Get control horizon.
  const auto& ch = horizons_[HorizonType::CONTROL];

  // Apply filter.
  const auto& ech = horizons_[HorizonType::ERODED_CONTROL];

  horizons_[HorizonType::YAW_GUIDANCE] = yawGuidance(
      static_cast<int>(col_d), ch.cols, p_.yaw_decay.left, p_.yaw_decay.right);
  const auto& yaw_guidance = horizons_[HorizonType::YAW_GUIDANCE];

  // Project throttles onto [r_min, r_max].
  const auto& throttle_h = horizons_[HorizonType::THROTTLE];

  // Compute guided throttle horizon.
  horizons_[HorizonType::GUIDED_THROTTLE] =
      throttleGuidance(throttle_h, yaw_guidance);
  const auto& guided_throttle_h = horizons_[HorizonType::GUIDED_THROTTLE];

  // Find the index of the desired control command.
  const auto control_idx = dampedMaxThrottleIndex(
      guided_throttle_h, p_.potential_inertia, static_cast<int>(col_d));

  // Compute yaw control command.
  const auto yaw =
      column2Yaw(throttle_h, static_cast<double>(control_idx) + 0.5,
                 p_.focal_length_x, p_.principal_point_x);

  // \TODO(CD-47) make separate constraint projection for yaw; for now, undo
  // translation.
  cmd.yaw = C_u_(cv::Point2d(p_.guidance_gains.yaw * yaw +
                                 p_.shape_parameters.translation,
                             0.0))
                .x;

  // Compute throttle control command (it is already projected by C_u_).
  const cv::Point2d throttle_set = throttle_h.at<cv::Point2d>(control_idx);
  const Interval_d interval(throttle_set.x, throttle_set.y);
  cmd.throttle = Interval_d::project_to_interval(interval, u_d.throttle);

  // Done.
  return rememberCommand(cmd);
}

//------------------------------------------------------------------------------

}  // namespace open_maeve
