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

#include "interval_constraints.h"

#include <limits>

#include "maeve_automation_core/maeve_geometry/comparisons.h"

namespace maeve_automation_core {
/** @brief This is used to return data from `simple_motion`. */
template <typename T>
struct SimpleDisplacement {
  const T distance;
  const T speed;

  friend bool operator==(const SimpleDisplacement& sd1,
                         const SimpleDisplacement& sd2) {
    return ((sd1.distance == sd2.distance) && (sd1.speed == sd2.speed));
  }

  friend bool operator!=(const SimpleDisplacement& sd1,
                         const SimpleDisplacement& sd2) {
    return !(sd1 == sd2);
  }
};

/** @brief Compute motion according to a simple, constant acceleration model. */
template <typename T>
SimpleDisplacement<T> simple_motion(const T& t, const T& v, const T& a) {
  const auto speed = (v + a * t);
  const auto distance = (t * (v + static_cast<T>(0.5) * a * t));
  return {distance, speed};
}

/**
 * @brief Under a constant acceleration model, compute the time until v1 and
 * v2 reach equilibrium.
 *
 * @note Time can be negative if relative speed became equal in the past but
 * will not in the future.
 *
 * @return The time at which v1 and v2 become equal, or NaN if they never do.
 */
template <typename T>
T time_to_zero_relative_speed(const T& v1, const T& v2, const T& a1,
                              const T& a2, const T& epsilon) {
  const auto numerator = (v1 - v2);
  const auto denominator = (a2 - a1);

  // Edge cases.
  if (approxZero(denominator, epsilon)) {
    if (approxZero(numerator, epsilon)) {
      return static_cast<T>(0);
    }
    return std::numeric_limits<T>::quiet_NaN();
  }

  return (numerator / denominator);
}
}  // namespace maeve_automation_core
