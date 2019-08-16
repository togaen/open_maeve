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

#include "utilities.h"

#include <cmath>
#include <limits>

#include <ros/console.h>

/**
 * @brief Check whether a varaible is NaN; return false immediately if so.
 *
 * @param var The variable to check.
 */
#define LOG_CHECK_NOT_NAN(var, logger)              \
  if (std::isnan(var)) {                            \
    logger(#var << " is NaN, which is not valid."); \
    return false;                                   \
  }
#define CHECK_NOT_NAN(var) LOG_CHECK_NOT_NAN(var, ROS_ERROR_STREAM)

/**
 * @brief Checks whether a value is finite; return false immediately on failure.
 *
 * @param var The variable to check.
 */
#define LOG_CHECK_FINITE(var, logger)                           \
  if (!std::isfinite(var)) {                                    \
    logger(#var << " is not finite: " << #var << " = " << var); \
    return false;                                               \
  }
#define CHECK_FINITE(var) LOG_CHECK_FINITE(var, ROS_ERROR_STREAM)

/**
 * @brief Checks whether a value is infinite; return false immediately on
 * failure.
 *
 * @param var The variable to check.
 */
#define LOG_CHECK_INFINITE(var, logger)                            \
  LOG_CHECK_NOT_NAN(var, logger);                                  \
  if (std::isfinite(var)) {                                        \
    logger(#var << " is finite or NaN: " << #var << " = " << var); \
    return false;                                                  \
  }
#define CHECK_INFINITE(var) LOG_CHECK_INFINITE(var, ROS_ERROR_STREAM)

/**
 * @brief Check that the value of 'var' is not equal to the value of 'val';
 * return false immediately if check fails.
 *
 * @param var The value to be checked.
 * @param val The value used to check 'var'.
 */
#define LOG_CHECK_NE(var, val, logger)                                      \
  if (var == val) {                                                         \
    logger(#var << " != " << #val << ": check failed for " << #var << " = " \
                << var << " and " << #val << " = " << val);                 \
    return false;                                                           \
  }
#define CHECK_NE(var, val) LOG_CHECK_NE(var, val, ROS_ERROR_STREAM)

/**
 * @brief Check that the value of 'var' is equal to the value of 'val'; return
 * false immediately if check fails.
 *
 * @param var The value to be checked.
 * @param val The value used to check 'var'.
 */
#define LOG_CHECK_EQ(var, val, logger)                                      \
  if (var != val) {                                                         \
    logger(#var << " == " << #val << ": check failed for " << #var << " = " \
                << var << " and " << #val << " = " << val);                 \
    return false;                                                           \
  }
#define CHECK_EQ(var, val) LOG_CHECK_EQ(var, val, ROS_ERROR_STREAM)

/**
 * @brief Check whether the given variable value is even; return false
 * immediately if check fails.
 *
 * @param var The variable to check.
 */
#define LOG_CHECK_EVEN(var, logger)                                         \
  if ((var % 2) != 0) {                                                     \
    logger(#var << " % 2 == 0: check failed for " << #var << " = " << var); \
    return false;                                                           \
  }
#define CHECK_EVEN(var) LOG_CHECK_EVEN(var, ROS_ERROR_STREAM)

/**
 * @brief Check whether the given variable value is odd; return false
 * immediately if check fails.
 *
 * @param var The variable to check.
 */
#define LOG_CHECK_ODD(var, logger)                                          \
  if ((var % 2) == 0) {                                                     \
    logger(#var << " % 2 != 0: check failed for " << #var << " = " << var); \
    return false;                                                           \
  }
#define CHECK_ODD(var) LOG_CHECK_ODD(var, ROS_ERROR_STREAM)

/**
 * @brief Check that var1 < var2; return false immediately if check fails.
 *
 * @param var1 First argument to comparison.
 * @param var2 Second argument to comparison.
 */
#define LOG_CHECK_LT(var1, var2, logger)                                      \
  if (!(var1 < var2)) {                                                       \
    logger(#var1 << " < " << #var2 << ": check failed for " << #var1 << " = " \
                 << var1 << ", " << #var2 << " = " << var2);                  \
    return false;                                                             \
  }
#define CHECK_LT(var1, var2) LOG_CHECK_LT(var1, var2, ROS_ERROR_STREAM)

/**
 * @brief Check that var1 <= var2; return false immediately if check fails.
 *
 * @param var1 First argument to comparison.
 * @param var2 Second argument to comparison.
 */
#define LOG_CHECK_LE(var1, var2, logger)                                       \
  if (!(var1 <= var2)) {                                                       \
    logger(#var1 << " <= " << #var2 << ": check failed for " << #var1 << " = " \
                 << var1 << ", " << #var2 << " = " << var2);                   \
    return false;                                                              \
  }
#define CHECK_LE(var1, var2) LOG_CHECK_LE(var1, var2, ROS_ERROR_STREAM)

/**
 * @brief Check that var1 > var2; return false immediately if check fails.
 *
 * @param var1 First argument to comparison.
 * @param var2 Second argument to comparison.
 */
#define LOG_CHECK_GT(var1, var2, logger)                                      \
  if (!(var1 > var2)) {                                                       \
    logger(#var1 << " > " << #var2 << ": check failed for " << #var1 << " = " \
                 << var1 << ", " << #var2 << " = " << var2);                  \
    return false;                                                             \
  }
#define CHECK_GT(var1, var2) LOG_CHECK_GT(var1, var2, ROS_ERROR_STREAM)

/**
 * @brief Check that var1 >= var2; return false immediately if check fails.
 *
 * @param var1 First argument to comparison.
 * @param var2 Second argument to comparison.
 */
#define LOG_CHECK_GE(var1, var2, logger)                                       \
  if (!(var1 >= var2)) {                                                       \
    logger(#var1 << " >= " << #var2 << ": check failed for " << #var1 << " = " \
                 << var1 << ", " << #var2 << " = " << var2);                   \
    return false;                                                              \
  }
#define CHECK_GE(var1, var2) LOG_CHECK_GE(var1, var2, ROS_ERROR_STREAM)

/**
 * @brief Check that var \in [r1, r2]; return false immediately if check fails.
 * @param var The value being checked.
 * @param r1 The initial point in the range.
 * @param r2 The terminal point in the range.
 */
#define LOG_CHECK_CONTAINS_CLOSED(var, r1, r2, logger) \
  LOG_CHECK_GE(var, r1, logger);                       \
  LOG_CHECK_LE(var, r2, logger);
#define CHECK_CONTAINS_CLOSED(var, r1, r2) \
  LOG_CHECK_CONTAINS_CLOSED(var, r1, r2, ROS_ERROR_STREAM)

/**
 * @brief Check that var \in (r1, r2); return false immediately if check fails.
 * @param var The value being checked.
 * @param r1 The initial point in the range.
 * @param r2 The terminal point in the range.
 */
#define LOG_CHECK_CONTAINS_OPEN(var, r1, r2, logger) \
  LOG_CHECK_GT(var, r1, logger);                     \
  LOG_CHECK_LT(var, r2, logger);
#define CHECK_CONTAINS_OPEN(var, r1, r2) \
  LOG_CHECK_CONTAINS_OPEN(var, r1, r2, ROS_ERROR_STREAM)

/**
 * @brief Check that container var is non-empty; return false immediately if
 * check fails.
 *
 * @param var
 */
#define LOG_CHECK_NONEMPTY(var, logger)       \
  if (var.empty()) {                          \
    logger(#var << ".empty(): check failed"); \
    return false;                             \
  }
#define CHECK_NONEMPTY(var) LOG_CHECK_NONEMPTY(var, ROS_ERROR_STREAM)
