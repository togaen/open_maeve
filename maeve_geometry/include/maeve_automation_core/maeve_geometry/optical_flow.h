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

#include <limits>

#include "maeve_automation_core/maeve_geometry/comparisons.h"

namespace maeve_automation_core {
/**
 * @brief Compute x or y component of optical flow.
 *
 * @note Translation speeds are w.r.t. the image plane and follow the sign
 *       convention defined by `tau_speed`.
 * @note Flow is undefined if the depth is zero. NaN is returned in this case.
 */
template <typename T>
T flow_component(const T& focal_length, const T& center, const T& depth,
                 const T& pixel_coordinate, const T& translation_speed_parallel,
                 const T& translation_speed_perpendicular, const T& epsilon) {
  if (approxZero(depth, epsilon)) {
    return std::numeric_limits<T>::quiet_NaN();
  }

  const auto center_relative_pixel = (pixel_coordinate - center);

  return ((focal_length / depth) *
          (translation_speed_parallel -
           (center_relative_pixel * -translation_speed_perpendicular)));
}
}  // namespace maeve_automation_core
