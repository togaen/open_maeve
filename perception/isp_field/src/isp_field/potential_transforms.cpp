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
cv::Scalar PotentialTransform<ConstraintType::HARD>::operator()(
    const cv::Scalar& pixel_value) const {
  auto return_value = pixel_value;

  // For convenience.
  const auto& p = shape_params_;

  // Compute potentials.
  if (pixel_value[0] < p.range_min) {
    // 0th order.
    return_value[0] = p.alpha * std::pow(p.range_min - pixel_value[0], -p.beta);
    // 1st order.
    return_value[1] = -p.alpha * p.beta * pixel_value[1] *
                      std::pow(p.range_min - pixel_value[0], -p.beta - 1.0);
  } else if (pixel_value[0] > p.range_max) {
    // 0th order.
    return_value[0] = p.alpha * std::pow(pixel_value[0] - p.range_max, -p.beta);
    // 1st order.
    return_value[1] = -p.alpha * p.beta * pixel_value[1] *
                      std::pow(pixel_value[0] - p.range_max, -p.beta - 1.0);
  } else {
    // 0th order.
    return_value[0] = INF;
    // 1st order.
    return_value[1] = 0.0;
  }

  // Done.
  return return_value;
}

template <>
cv::Scalar PotentialTransform<ConstraintType::SOFT>::operator()(
    const cv::Scalar& pixel_value) const {
  auto return_value = pixel_value;

  // For convenience.
  const auto& p = shape_params_;

  // Exponential term.
  const auto e_term = std::exp(-p.alpha * (pixel_value[0] - p.range_mid));

  // Compute 0th order potential.
  return_value[0] =
      p.range_min +
      (p.range_max - p.range_min) / std::pow(1.0 + e_term, 1.0 / p.beta);

  // Compute 1st order potential.
  const auto numerator = p.alpha * (p.range_max - p.range_min) * e_term;
  const auto denominator = p.beta * std::pow(1.0 + e_term, 1.0 + 1.0 / p.beta);
  return_value[1] = pixel_value[1] * numerator / denominator;

  // Done.
  return return_value;
}

}  // namespace maeve_automation_core
