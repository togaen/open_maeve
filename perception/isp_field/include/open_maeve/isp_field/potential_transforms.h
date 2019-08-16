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

#include <opencv2/opencv.hpp>

#include <iostream>

#include "open_maeve/isp_field/shape_parameters.h"

namespace open_maeve {
/** @brief Types of constraint transforms. */
enum class ConstraintType { HARD, SOFT };  // enum class ConstraintType

/**
 * @brief Overload output stream operator for ConstraintType enum.
 *
 * @param o The output stream.
 * @param c The constraint type enum value.
 *
 * @return The stream.
 */
std::ostream& operator<<(std::ostream& o, const ConstraintType c);

/**
 * @brief Template class for constrained potential value transforms.
 *
 * @tparam T The constraint type that the potential transform is modeling.
 */
template <ConstraintType T>
class PotentialTransform {
 public:
  /**
   * @brief Constructor: enable default construction.
   */
  PotentialTransform() = default;

  /**
   * @brief Constructor: Assign the interval constraint range.
   *
   * @param shape_params The shape parameters.
   */
  explicit PotentialTransform(const ShapeParameters& shape_params);

  /**
   * @brief Function definition for 0th order potential value transform.
   *
   * @param p The input pixel value.
   *
   * @return The potential value (index 0) and its time derivative (index 1).
   */
  cv::Point2d operator()(const cv::Point2d& p) const;

  /**
   * @brief Access the shape parameters for this transform.
   *
   * @return A const ref to the shape parameters.
   */
  const ShapeParameters& shapeParameters() const;

 private:
  /** @brief Shape parameter for this potential transform. */
  ShapeParameters p_;
};  // class PotentialTransform

template <ConstraintType T>
PotentialTransform<T>::PotentialTransform(const ShapeParameters& shape_params)
    : p_(shape_params) {}

template <ConstraintType T>
const ShapeParameters& PotentialTransform<T>::shapeParameters() const {
  return p_;
}

template <>
cv::Point2d PotentialTransform<ConstraintType::HARD>::operator()(
    const cv::Point2d& p) const;

template <>
cv::Point2d PotentialTransform<ConstraintType::SOFT>::operator()(
    const cv::Point2d& p) const;

}  // namespace open_maeve
