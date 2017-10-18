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

#include <ros/ros.h>

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
  }

/**
 * @brief Convenience macro for loading params into a ParamsBase object.
 *
 * @param param The address of the parameter on the parameter server.
 * @param var The name of the parameter and of the variable that holds it.
 */
#define LOAD_NAMED_PARAM(param, var)                                \
  if (!nh.getParam(param, var)) {                                   \
    ROS_ERROR_STREAM("Failed to load parameter '" << param << "'"); \
    return false;                                                   \
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
#define LOAD_STRUCT_PARAM(struct_name, var)                              \
  if (!nh.getParam(#var, struct_name.var)) {                             \
    ROS_ERROR_STREAM("Failed to load parameter '" << #struct_name << "." \
                                                  << #var << "'");       \
    return false;                                                        \
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
#define LOAD_NS_PARAM(ns, var)                                                \
  {                                                                           \
    auto ns_name = std::string(#ns);                                          \
    std::replace(ns_name.begin(), ns_name.end(), '.', '/');                   \
    if (!nh.getParam(ns_name + std::string("/") + std::string(#var),          \
                     ns.var)) {                                               \
      ROS_ERROR_STREAM("Failed to load parameter '" << ns_name << "/" << #var \
                                                    << "'");                  \
      return false;                                                           \
    }                                                                         \
  }
