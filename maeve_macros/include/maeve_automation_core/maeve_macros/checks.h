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

#include <cmath>
#include <limits>

#include <ros/console.h>

/**
 * @brief Check whether a varaible is NaN; return false immediately if so.
 *
 * @param var The variable to check.
 */
#define CHECK_NOT_NAN(var)                                    \
  if (std::isnan(var)) {                                      \
    ROS_ERROR_STREAM(#var << " is NaN, which is not valid."); \
    return false;                                             \
  }

/**
 * @brief Checks whether a value is finite; return false immediately on failure.
 *
 * @param var The variable to check.
 */
#define CHECK_FINITE(var)                                                 \
  if (!std::isfinite(var)) {                                              \
    ROS_ERROR_STREAM(#var << " is not finite: " << #var << " = " << var); \
    return false;                                                         \
  }

/**
 * @brief Checks whether a value is infinite; return false immediately on
 * failure.
 *
 * @param var The variable to check.
 */
#define CHECK_INFINITE(var)                                                  \
  CHECK_NOT_NAN(var);                                                        \
  if (std::isfinite(var)) {                                                  \
    ROS_ERROR_STREAM(#var << " is finite or NaN: " << #var << " = " << var); \
    return false;                                                            \
  }

/**
 * @brief Check that the value of 'var' is not equal to the value of 'val';
 * return false immediately if check fails.
 *
 * @param var The value to be checked.
 * @param val The value used to check 'var'.
 */
#define CHECK_NE(var, val)                                                     \
  if (var == val) {                                                            \
    ROS_ERROR_STREAM(#var << " != " << #val << ": check failed for " << #var   \
                          << " = " << var << " and " << #val << " = " << val); \
    return false;                                                              \
  }

/**
 * @brief Check that the value of 'var' is equal to the value of 'val'; return
 * false immediately if check fails.
 *
 * @param var The value to be checked.
 * @param val The value used to check 'var'.
 */
#define CHECK_EQ(var, val)                                                     \
  if (var != val) {                                                            \
    ROS_ERROR_STREAM(#var << " == " << #val << ": check failed for " << #var   \
                          << " = " << var << " and " << #val << " = " << val); \
    return false;                                                              \
  }

/**
 * @brief Check whether the given variable value is even; return false
 * immediately if check fails.
 *
 * @param var The variable to check.
 */
#define CHECK_EVEN(var)                                                      \
  if ((var % 2) != 0) {                                                      \
    ROS_ERROR_STREAM(#var << " % 2 == 0: check failed for " << #var << " = " \
                          << var);                                           \
    return false;                                                            \
  }

/**
 * @brief Check whether the given variable value is odd; return false
 * immediately if check fails.
 *
 * @param var The variable to check.
 */
#define CHECK_ODD(var)                                                       \
  if ((var % 2) == 0) {                                                      \
    ROS_ERROR_STREAM(#var << " % 2 != 0: check failed for " << #var << " = " \
                          << var);                                           \
    return false;                                                            \
  }

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
    ROS_ERROR_STREAM(#var1 << " <= " << #var2 << ": check failed for "         \
                           << #var1 << " = " << var1 << ", " << #var2 << " = " \
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
    ROS_ERROR_STREAM(#var1 << " >= " << #var2 << ": check failed for "         \
                           << #var1 << " = " << var1 << ", " << #var2 << " = " \
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
    ROS_ERROR_STREAM(#var << " >= 0: check failed for " << #var << " = " \
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
