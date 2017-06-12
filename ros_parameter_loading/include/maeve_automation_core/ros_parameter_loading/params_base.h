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
#pragma once

#include <ros/console.h>
#include <ros/ros.h>

#include <iostream>
#include <string>

#define CHECK_GT(var1, var2)                                                   \
  if (!(var1 > var2)) {                                                        \
    ROS_ERROR_STREAM(#var1 << " > " << #var2 << ": check failed for " << #var1 \
                           << " = " << var1 << ", " << #var2 << " = "          \
                           << var2);                                           \
    return false;                                                              \
  }

#define CHECK_STRICTLY_POSITIVE(var)                                     \
  if (var <= 0) {                                                        \
    ROS_ERROR_STREAM(#var << " <= 0: check failed for " << #var << " = " \
                          << var);                                       \
    return false;                                                        \
  }

#define CHECK_NONEMPTY(var)                             \
  if (var.empty()) {                                    \
    ROS_ERROR_STREAM(#var << ".empty(): check failed"); \
    return false;                                       \
  }

#define LOAD_PARAM(var)                                            \
  if (!nh.getParam(#var, var)) {                                   \
    ROS_ERROR_STREAM("Failed to load parameter '" << #var << "'"); \
    return false;                                                  \
  }                                                                \
  {                                                                \
    std::stringstream ss;                                          \
    ss << #var << ": " << var << "\n";                             \
    loaded_param_set += ss.str();                                  \
  }

namespace maeve_automation_core {

struct ParamsBase {
  /// \brief Stream overload
  friend std::ostream& operator<<(std::ostream& os, const ParamsBase& params) {
    return os << params.loaded_param_set;
  }

  /// \brief Load parameters from parameter server.
  virtual bool load(const ros::NodeHandle& nh) = 0;

 protected:
  /// \brief Human-readable string of parameters loaded by load() function.
  std::string loaded_param_set;
};  // struct StruckVisualTrackingParams

}  // namespace maeve_automation_core
