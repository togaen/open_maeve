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

#define _USE_MATH_DEFINES
#include <cmath>

namespace maeve_automation_core {
/**
 * @brief Compute the value:
 *
 *     sgn(val) * sqrt(|val|)
 *
 * @note This is useful as a shaping function in heuristics.
 */
template <typename T>
T sqrt_magnitude(const T& val) {
  return std::copysign(std::sqrt(std::abs(val)), val);
}

/**
 * @brief Utilities for degrees <> radians
 * @{
 */
template <typename T>
constexpr T deg_2_rad(const T& deg) {
  return (deg * T(M_PI / 180.0));
}
template <typename T>
constexpr T rad_2_deg(const T& rad) {
  return (rad * T(180.0 / M_PI));
}
/** @} */

/** @brief Utility function useful for readability. */
template <typename T>
T inverse(const T& val) {
  return (static_cast<T>(1) / val);
}

/** @brief Make axis conventions readable in the code. */
template <typename T>
T invert_axis(const T& axis) {
  return -axis;
}

/**
 * @brief Get a number with magnitude of 'value' and sign determined by 'sign'
 *
 * @return -|value| if sign is true; |value| otherwise
 */
template <typename T>
T signed_value(const T& value, const bool sign) {
  const auto num_sign = (sign ? T(-1) : T(1));
  return std::copysign(value, num_sign);
}

/**
 * @brief Convenience method for performing logical exclusive or ops.
 *
 * @tparam T The input type; this must be convertible to bool.
 *
 * @param a The first value.
 * @param b The second value.
 *
 * @return True if exactly one of 'a' and 'b' is true; otherwise false.
 */
template <typename T>
inline bool exclusiveOr(const T& a, const T& b) {
  return (static_cast<bool>(a) != static_cast<bool>(b));
}

/**
 * @brief Check for approximate equality in absolute terms.
 *
 * @tparam T The type of value to check.
 *
 * @param a The first value in comparison.
 * @param b The second value in comparison.
 * @param eps The absolute range within which to assume equality.
 *
 * @return True if 'a' and 'b' are within 'eps' of each other; otherwise false.
 */
template <typename T>
inline bool approxEq(const T& a, const T& b, const T& eps) {
  // TODO(me): make this use relative magnitude comparison
  return std::abs(a - b) <= eps;
}

/**
 * @brief Check for approximate inequality in absolute terms.
 *
 * @tparam T The type of value to check.
 *
 * @param a The first value in comparison.
 * @param b The second value in comparison.
 * @param eps The absolute range within which to assume inequality.
 *
 * @return True if 'a' and 'b' are outside of 'eps' of each other; otherwise
 * false.
 */
template <typename T>
inline bool approxNe(const T& a, const T& b, const T& eps) {
  return !approxEq(a, b, eps);
}

/**
 * @brief Check whether a value is within epsilon of zero.
 *
 * @tparam T The type of the value to check.
 *
 * @param a The value to check.
 * @param eps The absolute range within with to assume equality.
 *
 * @return True if 'a' is within 'eps' of zero; otherwise false.
 */
template <typename T>
inline bool approxZero(const T& a, const T& eps) {
  return approxEq(a, static_cast<T>(0), eps);
}

/**
 * @brief If 'val' is within 'espilon' of zero, return zero. Otherwise return
 * 'val'.
 *
 * @tparam T The type of the arguments.
 * @param val The value being clamped.
 * @param epsilon The range about zero to perform a clamp.
 *
 * @return Zero if val is within epsilon of zero; otherwise val.
 */
template <typename T>
inline T clampToZero(const T& val, const T& eps) {
  return (approxZero(val, eps) ? static_cast<T>(0) : val);
}

/**
 * @brief Check for approximate less than in absolute terms.
 *
 * @tparam T The type of value to check.
 *
 * @param a The first value in comparison.
 * @param b The second value in comparison.
 * @param eps The absolute range within which to assume less than.
 *
 * @return True if 'a' is less than 'b' minus 'eps'; otherwise false.
 */
template <typename T>
inline bool approxLt(const T& a, const T& b, const T& eps) {
  return (approxNe(a, b, eps) && (a < b));
}

/**
 * @brief Check for approximate less than or equal in absolute terms.
 *
 * @tparam T The type of value to check.
 *
 * @param a The first value in comparison.
 * @param b The second value in comparison.
 * @param eps The absolute range within which to assume less than or equal.
 *
 * @return True if 'a' is less than or equal to 'b' plus 'eps'; otherwise false.
 */
template <typename T>
inline bool approxLe(const T& a, const T& b, const T& eps) {
  return (approxEq(a, b, eps) || (a < b));
}

/**
 * @brief Check for approximate greater than or equal in absolute terms.
 *
 * @tparam T The type of value to check.
 *
 * @param a The first value in comparison.
 * @param b The second value in comparison.
 * @param eps The absolute range within which to assume greater than or equal.
 *
 * @return True if 'a' is greater than or equal to 'b' minus 'eps'; otherwise
 * false.
 */
template <typename T>
inline bool approxGe(const T& a, const T& b, const T& eps) {
  return !approxLt(a, b, eps);
}

/**
 * @brief Check for approximate greater than in absolute terms.
 *
 * @tparam T The type of value to check.
 *
 * @param a The first value in comparison.
 * @param b The second value in comparison.
 * @param eps The absolute range within which to assume greater than.
 *
 * @return True if 'a' is greater than 'b' minus 'eps'; otherwise false.
 */
template <typename T>
inline bool approxGt(const T& a, const T& b, const T& eps) {
  return !approxLe(a, b, eps);
}
}  // namespace maeve_automation_core
