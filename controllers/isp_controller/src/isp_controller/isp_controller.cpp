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

#include <limits>

#include "isp_controller/lib.h"

namespace maeve_automation_core {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

ISP_Controller::Params::Params()
    : K_P(NaN),
      K_D(NaN),
      kernel_width(-1),
      kernel_height(-1),
      kernel_horizon(-1) {}

ISP_Controller::ControlCommand::ControlCommand()
    : throttle(NaN), steering(NaN) {}

ISP_Controller::ControlCommand::ControlCommand(const double t, const double s)
    : throttle(t), steering(s) {}

ISP_Controller::ISP_Controller(const Params& params,
                               const ControlCommand& initial_commanded_control)
    : p_(params), commanded_control_(initial_commanded_control) {}

ISP_Controller::ControlCommand ISP_Controller::computeControlCommand(
    const cv::Mat& ISP) {
  ControlCommand cmd;

  // Get control horizon.
  const auto C_u =
      PotentialTransform<ConstraintType::SOFT>(p_.shape_parameters);
  const cv::Mat safe_controls =
      safeControls(ISP, C_u, p_.kernel_width, p_.kernel_height,
                   p_.kernel_horizon, p_.K_P, p_.K_D);

  // Compute steering modes.
  const auto index_pairs = computeHorizonMinima(safe_controls);

  // Choose mode.

  // Compute control command.

  // Done.
  return cmd;
}
}  // namespace maeve_automation_core
