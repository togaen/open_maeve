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

/**
 * @brief Check that var1 < var2; return false immediately if check fails.
 *
 * @param var1 First argument to comparison.
 * @param var2 Second argument to comparison.
 */
#define CHECK_LT(var1, var2)                                                   \
  if (!(var1 < var2)) {                                                        \
    ROS_ERROR_STREAM(#var1 << " < " << #var2 << ": check failed for " << #var1 \
                           << " = " << var1 << ", " << #var2 << " = "          \
                           << var2);                                           \
    return false;                                                              \
  }

/**
 * @brief Check that var1 <= var2; return false immediately if check fails.
 *
 * @param var1 First argument to comparison.
 * @param var2 Second argument to comparison.
 */
#define CHECK_LE(var1, var2)                                                   \
  if (!(var1 <= var2)) {                                                       \
    ROS_ERROR_STREAM(#var1 << " <= " << #var2 << ": check failed for " << #var1 \
                           << " = " << var1 << ", " << #var2 << " = "          \
                           << var2);                                           \
    return false;                                                              \
  }

/**
 * @brief Check that var1 > var2; return false immediately if check fails.
 *
 * @param var1 First argument to comparison.
 * @param var2 Second argument to comparison.
 */
#define CHECK_GT(var1, var2)                                                   \
  if (!(var1 > var2)) {                                                        \
    ROS_ERROR_STREAM(#var1 << " > " << #var2 << ": check failed for " << #var1 \
                           << " = " << var1 << ", " << #var2 << " = "          \
                           << var2);                                           \
    return false;                                                              \
  }

/**
 * @brief Check that var1 >= var2; return false immediately if check fails.
 *
 * @param var1 First argument to comparison.
 * @param var2 Second argument to comparison.
 */
#define CHECK_GE(var1, var2)                                                   \
  if (!(var1 >= var2)) {                                                       \
    ROS_ERROR_STREAM(#var1 << " >= " << #var2 << ": check failed for " << #var1 \
                           << " = " << var1 << ", " << #var2 << " = "          \
                           << var2);                                           \
    return false;                                                              \
  }

/**
 * @brief Check that var \in [r1, r2]; return false immediately if check fails.
 * @param var The value being checked.
 * @param r1 The initial point in the range.
 * @param r2 The terminal point in the range.
 */
#define CHECK_CONTAINS_CLOSED(var, r1, r2) \
  CHECK_GE(var, r1);                       \
  CHECK_LE(var, r2);

/**
 * @brief Check that var \in (r1, r2); return false immediately if check fails.
 * @param var The value being checked.
 * @param r1 The initial point in the range.
 * @param r2 The terminal point in the range.
 */
#define CHECK_CONTAINS_OPEN(var, r1, r2) \
  CHECK_GT(var, r1);                     \
  CHECK_LT(var, r2);

/**
 * @brief Check that var > 0; return false immediately if check fails.
 *
 * @param var The value to check.
 */
#define CHECK_STRICTLY_POSITIVE(var)                                     \
  if (var <= 0) {                                                        \
    ROS_ERROR_STREAM(#var << " <= 0: check failed for " << #var << " = " \
                          << var);                                       \
    return false;                                                        \
  }

/**
 * @brief Check that container var is non-empty; return false immediately if
 * check fails.
 *
 * @param var
 */
#define CHECK_NONEMPTY(var)                             \
  if (var.empty()) {                                    \
    ROS_ERROR_STREAM(#var << ".empty(): check failed"); \
    return false;                                       \
  }

/**
 * @brief Convenience macro for loading params into a ParamsBase object.
 *
 * 'var' is a member variable of the ParamsBase object, and it must have
 * 'exactly' the same name as the ROS parameter. Return false immediately if
 * loading fails. Otherwise, append the name and value of var to the
 * loaded_param_set string in the ParamsBase object.
 *
 * @param var The name of the parameter and of the variable that holds it.
 */
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

/**
 * @brief Convenience macro for loading params into a ParamsBase object.
 *
 * 'var' is a member variable of a struct named 'struct_name', which belongs to
 * the ParamsBase object. 'var' must have 'exactly' the same name as the ROS
 * parameter. Return false immediately if loading fails. Otherwise, append the
 * name and value of var to the loaded_param_set string in the ParamsBase
 * object.
 *
 * @param struct_name The name of the struct that 'var' belongs to.
 * @param var The name of the parameter and of the variable that holds it.
 */
#define LOAD_STRUCT_PARAM(struct_name, var)                               \
  if (!nh.getParam(#var, struct_name.var)) {                              \
    ROS_ERROR_STREAM("Failed to load parameter '" << #struct_name << "."  \
                                                  << #var << "'");        \
    return false;                                                         \
  }                                                                       \
  {                                                                       \
    std::stringstream ss;                                                 \
    ss << #struct_name << "." << #var << ": " << struct_name.var << "\n"; \
    loaded_param_set += ss.str();                                         \
  }

/**
 * @brief Convenience macro for loading scoped params into a ParamsBase object.
 *
 * 'var' is a member of an object 'ns' in the ParamsBase object, where 'var'
 * must have 'exactly' the same name as the ROS parameter, and 'ns' must have
 * 'exactly' the same name as the namespace of the 'var' parameter. Return
 * false immediately if loading fails. Otherwise, append the name and value of
 * ns.var to the loaded_param_set string in the ParamsBase object.

 * @param ns The namespace (or scope) of var in the parameter server.
 * @param var The name of the parameter and of the variable that holds it.
 */
#define LOAD_NS_PARAM(ns, var)                                              \
  if (!nh.getParam(std::string(#ns) + std::string("/") + std::string(#var), \
                   ns.var)) {                                               \
    ROS_ERROR_STREAM("Failed to load parameter '" << #ns << "/" << #var     \
                                                  << "'");                  \
    return false;                                                           \
  }                                                                         \
  {                                                                         \
    std::stringstream ss;                                                   \
    ss << #ns << "." << #var << ": " << ns.var << "\n";                     \
    loaded_param_set += ss.str();                                           \
  }

namespace maeve_automation_core {

/** @brief An interface for objects that load and contain ROS parameters.*/
struct ParamsBase {
  /**
   * @brief Write a parameter object to a string stream.
   *
   * @param os The output stream.
   * @param params The parameter object.
   *
   * @return The stream with the serialized parameter object.
   */
  friend std::ostream& operator<<(std::ostream& os, const ParamsBase& params) {
    return os << params.loaded_param_set;
  }

  /**
   * @brief Load parameters from the ROS parameter server.
   *
   * @param nh The ROS node handle.
   *
   * @return True if loading passed sanity check; otherwise false.
   */
  virtual bool load(const ros::NodeHandle& nh) = 0;

 protected:
  /**
   * @brief Human-readable string of parameters loaded by load() function.
   *
   * @note This is for informational use only, it is not guaranteed to be in
   * sync with the member values.
   */
  std::string loaded_param_set;
};  // struct StruckVisualTrackingParams

}  // namespace maeve_automation_core
