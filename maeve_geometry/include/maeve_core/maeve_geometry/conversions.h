/*
 * Copyright 2019 Maeve Automation
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
#include <limits>

namespace maeve_core {
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
  return (deg * static_cast<T>(M_PI / 180.0));
}
template <typename T>
constexpr T rad_2_deg(const T& rad) {
  return (rad * static_cast<T>(180.0 / M_PI));
}
/** @} */

/**
 * @brief Utility function useful for readability.
 *
 * @note In real-valued arithmetic, division by zero is undefined, so this
 * function returns NaN in such cases. This is different behavior than IEEE
 * floating point arithmetic which affinely extends the real number line and
 * applies limiting behavior.
 */
template <typename T>
T inverse(const T& val) {
  if (val == static_cast<T>(0)) {
    return std::numeric_limits<T>::quiet_NaN();
  }
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
  const auto num_sign = (sign ? static_cast<T>(-1) : static_cast<T>(1));
  return std::copysign(value, num_sign);
}
}  // namespace maeve_core
