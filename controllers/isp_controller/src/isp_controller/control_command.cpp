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
#include "maeve_automation_core/isp_controller/control_command.h"

#include <cmath>
#include <iostream>
#include <limits>

#include "controller_interface_msgs/Command2D.h"

namespace maeve_automation_core {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

std::ostream& operator<<(std::ostream& o, const ControlCommand& u) {
  return o << "{" << u.throttle << ", " << u.yaw << "}";
}

bool ControlCommand::valid() const {
  const auto throttle_okay = !std::isnan(throttle) && std::isfinite(throttle);
  const auto yaw_okay = !std::isnan(yaw) && std::isfinite(yaw);
  return throttle_okay && yaw_okay;
}

ControlCommand::ControlCommand() : throttle(NaN), yaw(NaN) {}

ControlCommand::ControlCommand(const double t, const double y)
    : throttle(t), yaw(y) {}
}  // namespace maeve_automation_core
