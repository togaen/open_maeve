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
#include "maeve_automation_core/cisp_controller/cisp_controller.h"

#include <limits>

namespace maeve_automation_core {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

CISP_Controller::ControlCommand::ControlCommand()
    : throttle(NaN), steering(NaN) {}

cv::Mat projectCISP(const cv::Mat& CISP) {
  cv::Mat control_horizon = cv::Mat::zeros(1, CISP.cols, CV_64FC2);

  return control_horizon;
}

CISP_Controller::ControlCommand CISP_Controller::computeControlCommand(
    const cv::Mat& CISP) {
  ControlCommand cmd;

  // Get control horizon.
  cv::Mat control_horizon = projectCISP(CISP);

  // Compute steering modes.

  // Choose mode.

  // Compute control command.

  // Done.
  return cmd;
}
}  // namespace maeve_automation_core
