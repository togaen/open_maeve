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

#include <tuple>

namespace maeve_automation_core {
/** @brief Typedef the constraint range type for convenience. */
typedef std::tuple<double, double> CRange;

/** @brief Types of constraint transforms. */
enum class ConstraintType { HARD, SOFT };  // enum class ConstraintType

/**
 * @brief Template class for constrained potential value transforms.
 *
 * @tparam T The constraint type that the potential transform is modeling.
 */
template <ConstraintType T>
struct PotentialTransform {
  /** @brief The closed interval contraint range. */
  CRange c_range;

  /**
   * @brief Constructor: Assign the interval constraint range.
   *
   * @param c_r The interval constraint range.
   */
  PotentialTransform(const CRange& c_r);

  /**
   * @brief Function definition for 0th order potential value transform.
   *
   * @param p The input pixel value.
   *
   * @return The potential value (index 0) and its time derivative (index 1).
   */
  cv::Scalar operator()(const cv::Scalar& p) const;
};  // struct ConstraintTransform

template <ConstraintType T>
PotentialTransform<T>::PotentialTransform(const CRange& c_r) : c_range(c_r) {}

template <>
cv::Scalar PotentialTransform<ConstraintType::HARD>::operator()(
    const cv::Scalar& p) const;

template <>
cv::Scalar PotentialTransform<ConstraintType::SOFT>::operator()(
    const cv::Scalar& p) const;

}  // namespace maeve_automation_core
