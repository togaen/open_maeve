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
#include "maeve_automation_core/isp_field/potential_transforms.h"

#include <cmath>
#include <limits>

namespace maeve_automation_core {
namespace {
static const auto INF = std::numeric_limits<double>::infinity();
}  // namespace

std::ostream& operator<<(std::ostream& o, const ConstraintType c) {
  switch (c) {
    case ConstraintType::HARD:
      return o << "ConstraintType::HARD";
    case ConstraintType::SOFT:
      return o << "ConstraintType::SOFT";
    default:
      return o << "String not defined for enum value: " << c;
      break;
  }
}

template <>
cv::Point2d PotentialTransform<ConstraintType::HARD>::operator()(
    const cv::Point2d& pixel_value) const {
  auto return_value = pixel_value;

  // Compute potentials.
  if (pixel_value.x < p_.range_min) {
    // 0th order.
    return_value.x =
        -p_.alpha * std::pow(p_.range_min - pixel_value.x, -p_.beta);
    // 1st order.
    return_value.y = p_.alpha * p_.beta * pixel_value.y *
                     std::pow(p_.range_min - pixel_value.x, -p_.beta - 1.0);
  } else if (pixel_value.x > p_.range_max) {
    // 0th order.
    return_value.x =
        -p_.alpha * std::pow(pixel_value.x - p_.range_max, -p_.beta);
    // 1st order.
    return_value.y = p_.alpha * p_.beta * pixel_value.y *
                     std::pow(pixel_value.x - p_.range_max, -p_.beta - 1.0);
  } else {
    // 0th order.
    return_value.x = -INF;
    // 1st order.
    return_value.y = 0.0;
  }

  // Done.
  return return_value;
}

template <>
cv::Point2d PotentialTransform<ConstraintType::SOFT>::operator()(
    const cv::Point2d& pixel_value) const {
  auto return_value = pixel_value;

  // Exponential term.
  const auto e_term = std::exp(-p_.alpha * (pixel_value.x - p_.rangeMidPoint()));

  // Compute 0th order potential.
  return_value.x =
      p_.range_min +
      (p_.range_max - p_.range_min) / std::pow(1.0 + e_term, 1.0 / p_.beta);

  // Compute 1st order potential.
  const auto numerator = p_.alpha * (p_.range_max - p_.range_min) * e_term;
  const auto denominator =
      p_.beta * std::pow(1.0 + e_term, 1.0 + 1.0 / p_.beta);
  return_value.y = pixel_value.y * numerator / denominator;

  // Done.
  return return_value;
}

}  // namespace maeve_automation_core
