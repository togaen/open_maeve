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

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <functional>

#include "maeve_automation_core/maeve_geometry/interval.h"

namespace maeve_automation_core {

/**
 * @brief Axis-aligned Bounding Box with compile-tome specified dimensionality.
 *
 * @note Behavior is undefined for Dim = 0. This class simply generalizes the
 * Interval type to higher dimensions.
 *
 * @tparam Dim Dimensionality of box specified at compile time.
 */
template <int Dim>
class AABB {
 public:
  /**
   * @brief Default constructor: rely on default constructor of Interval.
   */
  AABB();

  /**
   * @brief Explicit constructor: Pass an array of intervals.
   *
   * @param bounds The ordered array of axis bounds.
   */
  explicit AABB(std::array<Interval, Dim>&& bounds);

  /**
   * @brief The minimum bound of the AABB along a given axis.
   *
   * @param aabb The AABB to retrieve the min bound from.
   * @param axis The index of the axis to retrieve the min bound from.
   *
   * @return The minimum bound.
   */
  static double min(const AABB& aabb, const int axis);

  /**
   * @brief The maximum bound of the AABB along a give axis.
   *
   * @param aabb The AABB to retrieve the max bound from.
   * @param axis The index of the axis to retrieve the max bound from.
   *
   * @return The maximum bound.
   */
  static double max(const AABB& aabb, const int axis);

  /**
   * @brief Return the volume of the AABB.
   *
   * @note For empty or invalid AABB, NaN is returned. See Interval::empty
   * for note on distinction between length zero and the emptiness property.
   *
   * @param interval
   *
   * @return The length of the interval.
   */
  static double volume(const AABB& aabb);

  /**
   * @brief Whether the AABB is empty or not.
   *
   * @note See note at Interval::empty
   *
   * @param aabb The AABB to test for emptiness.
   *
   * @return True if the AABB is empty; otherwise false.
   */
  static bool empty(const AABB& aabb);

  /**
   * @brief Whether the AABB is valid.
   *
   * @note An AABB with NaN in the bounds is only valid if it is empty.
   *
   * @param aabb The AABB to test for validity.
   *
   * @return True if empty or min <= max for all axis bounds; otherwise false.
   */
  static bool valid(const AABB& aabb);

  /**
   * @brief Compute the convex hull of two AABBs as a new AABB.
   *
   * @param aabb1 The first AABB.
   * @param aabb2 The second AABB.
   *
   * @return The newly constructed AABB hull.
   */
  static AABB convexHull(const AABB& aabb1, const AABB& aabb2);

  /**
   * @brief Test for whether a given AABB contains a given value.
   *
   * @param aabb The AABB.
   * @param point The point to check for containment.
   *
   * @return True if 'aabb' contains 'value'; otherwise false.
   */
  static bool contains(const AABB& aabb,
                       const Eigen::Matrix<double, Dim, 1>& point);

  /**
   * @brief Compute the intersection of two AABBs as a new AABB.
   *
   * @param aabb1 The first AABB.
   * @param aabb2 The second AABB.
   *
   * @return The newly constructed AABB intersection.
   */
  static AABB intersect(const AABB& aabb1, const AABB& aabb2);

 private:
  /** @brief An array of axis bounds that define the AABB. */
  std::array<Interval, Dim> axis_bounds_;
};  // class AABB

template <int Dim>
AABB<Dim>::AABB() {
  static_assert(Dim > 0, "AABBs must have strictly positive dimensionality.");
}

template <int Dim>
AABB<Dim>::AABB(std::array<Interval, Dim>&& bounds)
    : axis_bounds_(std::move(bounds)) {
  static_assert(Dim > 0, "AABBs must have strictly positive dimensionality.");
}

template <int Dim>
double AABB<Dim>::min(const AABB<Dim>& aabb, const int axis) {
  // Use 'at' accessor do enforce bounds checking.
  return Interval::min(aabb.axis_bounds_.at(axis));
}

template <int Dim>
double AABB<Dim>::max(const AABB<Dim>& aabb, const int axis) {
  // Use 'at' accessor do enforce bounds checking.
  return Interval::max(aabb.axis_bounds_.at(axis));
}

template <int Dim>
double AABB<Dim>::volume(const AABB& aabb) {
  auto v = 1.0;
  return std::accumulate(aabb.axis_bounds_.begin(), aabb.axis_bounds_.end(), v,
                         [](const double l, const Interval& i) {
                           return l * Interval::length(i);
                         });
}

template <int Dim>
bool AABB<Dim>::empty(const AABB<Dim>& aabb) {
  auto is_empty = aabb.axis_bounds_.empty();
  std::for_each(
      std::begin(aabb.axis_bounds_), std::end(aabb.axis_bounds_),
      [&](const Interval& i) { is_empty = is_empty || Interval::empty(i); });
  return is_empty;
}

template <int Dim>
bool AABB<Dim>::valid(const AABB<Dim>& aabb) {
  auto is_valid = true;
  std::for_each(
      std::begin(aabb.axis_bounds_), std::end(aabb.axis_bounds_),
      [&](const Interval& i) { is_valid = is_valid && Interval::valid(i); });
  return is_valid;
}

template <int Dim>
bool AABB<Dim>::contains(const AABB<Dim>& aabb,
                         const Eigen::Matrix<double, Dim, 1>& point) {
  auto interior = true;
  for (auto i = 0; i < Dim; ++i) {
    interior = interior && Interval::contains(aabb.axis_bounds_[i], point[i]);
  }
  return interior;
}

template <int Dim>
AABB<Dim> AABB<Dim>::intersect(const AABB<Dim>& aabb1, const AABB<Dim>& aabb2) {
  std::array<Interval, Dim> bounds;
  for (auto i = 0; i < Dim; ++i) {
    bounds[i] =
        Interval::intersect(aabb1.axis_bounds_[i], aabb2.axis_bounds_[i]);
  }
  return AABB<Dim>(std::move(bounds));
}

template <int Dim>
AABB<Dim> AABB<Dim>::convexHull(const AABB<Dim>& aabb1,
                                const AABB<Dim>& aabb2) {
  std::array<Interval, Dim> bounds;
  for (auto i = 0; i < Dim; ++i) {
    bounds[i] =
        Interval::convexHull(aabb1.axis_bounds_[i], aabb2.axis_bounds_[i]);
  }
  return AABB<Dim>(std::move(bounds));
}

}  // namespace maeve_automation_core
