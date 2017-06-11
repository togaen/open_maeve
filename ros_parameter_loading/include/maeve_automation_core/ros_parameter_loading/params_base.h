// Copyright 2017 Maeve Automation
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
