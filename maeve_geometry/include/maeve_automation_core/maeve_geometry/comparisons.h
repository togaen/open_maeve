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

namespace maeve_automation_core {
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
