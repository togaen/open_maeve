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

#include <iostream>
#include <limits>
#include <tuple>

#include <boost/optional.hpp>

namespace maeve_automation_core {
/**
 * @brief Data structure to contain scalar interval bounds.
 *
 * @note An exception is thrown on construction if the provided interval bounds
 * are invalid (i.e., if min > max).
 */
class Interval {
 public:
  /** @name Special interval bound values
   *
   * These values are used to construct special types of intervals.
   */
  static constexpr auto MIN = std::numeric_limits<double>::lowest();
  static constexpr auto MAX = std::numeric_limits<double>::max();
  static constexpr auto SUPREMUM = std::numeric_limits<double>::infinity();
  static constexpr auto INFIMUM = -SUPREMUM;
  /** @} */

  /** @name Comparison operations
   *
   * Comparison operations for Interval types.
   *
   * @note Like comparisons of NaN, invalid intervals always compare false.
   *
   * @{
   */

  /**
   * @brief Compute equality.
   *
   * Two intervals compare equal iff they are both valid and they are both
   * either empty or have equal bounds.
   *
   * @param interval1 The first interval to compare.
   * @param interval2 The second interval to compare.
   *
   * @return True if the intervals compare equal; otherwise false.
   */
  friend bool operator==(const Interval& interval1, const Interval& interval2);

  /**
   * @brief Compute inequality.
   *
   * This always returns the negation of the == operator.
   *
   * @param interval1 The first interval to compare.
   * @param interval2 the second interval to compare.
   *
   * @return True if the intervals do not compare equal; otherwise false.
   */
  friend bool operator!=(const Interval& interval1, const Interval& interval2);

  /** }@ */

  /**
   * @brief Stream overload for Interval types.
   *
   * @note Output precision is fixed inside the function definition, and the
   * serialization is JSON compatible.
   *
   * @param os The output stream.
   * @param interval The interval to serialize.
   *
   * @return The output stream with the serialized interval.
   */
  friend std::ostream& operator<<(std::ostream& os, const Interval& interval);

  /**
   * @brief The minimum bound of the interval.
   *
   * @param interval The interval to retrieve the min bound from.
   *
   * @return The minimum bound.
   */
  static double min(const Interval& interval);

  /**
   * @brief The maximum bound of the interval.
   *
   * @param interval The interval to retrieve the max bound from.
   *
   * @return The maximum bound.
   */
  static double max(const Interval& interval);

  /**
   * @brief Utility to retrieve both interval bounds at once.
   *
   * @param interval The interval to retrieve the bounds from.
   *
   * @return A tuple of the bounds.
   */
  static const std::tuple<double, double>& bounds(const Interval& interval);

  /**
   * @brief Return the length of the interval.
   *
   * @note For empty or invalid intervals, NaN is returned. See Interval::empty
   * for note on distinction between length zero and the emptiness property.
   *
   * @param interval The interval to check.
   *
   * @return The length of the interval.
   */
  static double length(const Interval& interval);

  /**
   * @brief Utility for checking whether an interval has zero length.
   *
   * @note For empty or invalid intervals, false is returned. See
   * Interval::empty for note on distinction between length zero and the
   * emptiness property.
   *
   * @param interval The interval to check.
   *
   * @return True if the interval has zero length; otherwise false.
   */
  static bool zeroLength(const Interval& interval);

  /**
   * @brief Whether the interval is empty or not.
   *
   * @note Emptiness refers to whether the interval contains any points and is
   * thus a distinct property from length: an interval is non-empty if contains
   * only a single point even though its length in that case is zero.
   *
   * @param interval The interval to test for emptiness.
   *
   * @return True if the interval is empty; otherwise false.
   */
  static bool empty(const Interval& interval);

  /**
   * @brief Test for whether a given interval contains a given value.
   *
   * @param interval The interval.
   * @param value The value.
   *
   * @return True if 'interval' contains 'value'; otherwise false.
   */
  static bool contains(const Interval& interval, const double value);

  /**
   * @brief Test for whether 'interval1' \subseteq 'interval2'
   *
   * @param interval1 The first interval.
   * @param interval2 The second interval.
   *
   * @return True if interval1 \subseteq interval2; otherwise false.
   */
  static bool isSubsetEq(const Interval& interval1, const Interval& interval2);

  /**
   * @brief Compute the intersection of two intervals as a new interval.
   *
   * @param interval1 The first interval.
   * @param interval2 The second interval.
   *
   * @return The newly constructed interval intersection.
   */
  static Interval intersect(const Interval& interval1,
                            const Interval& interval2);

  /**
   * @brief Test whether two intervals overlap.
   *
   * @note If either interval is empty, this returns true.
   */
  static bool overlap(const Interval& interval1, const Interval& interval2);

  /**
   * @brief Project a scalar 'val' onto 'interval'.
   *
   * @param interval The interval to project onto.
   * @param val The value to project.
   *
   * @return If 'val' \in 'interval', return 'val'; otherwise return the nearer
   * interval bound.
   */
  static double projectToInterval(const Interval& interval, const double val);

  /**
   * @brief Compute the scaling along the interval that corresponds to 'val'
   *
   * @returns s \in [0, 1] for val \in interval, or s < 0 or s > 1 for val < min
   * or val > max
   */
  static double scale_to_interval(const Interval& interval, const double val);

  /**
   * @brief Compute the convex hull of two intervals as a new interval.
   *
   * @note For intervals that overlap, this is equivalent to a union.
   *
   * @param interval1 The first interval.
   * @param interval2 The second interval.
   *
   * @return The newly constructed interval hull.
   */
  static Interval convexHull(const Interval& interval1,
                             const Interval& interval2);

  /**
   * @brief Compute the interval that exactly contains the input intervals.
   *
   * This method implements interval union. It computes and returns a convex
   * hull of two intervals iff they intersect. If the two intervals do not
   * intersect, there is no interval that exactly contains the input intervals,
   * so an invalid interval is returned.
   *
   * @param interval1 The first interval to merge.
   * @param interval2 The second interval to merge.
   *
   * @return The merged interval.
   */
  static boost::optional<Interval> merge(const Interval& interval1,
                                         const Interval& interval2);

  /**
   * @brief Factory method to build the set of affinely extended reals.
   *
   * @return [-\infty, +\infty]
   */
  static Interval affinelyExtendedReals();

  /**
   * @brief Factory method to build the set of machien representable reals.
   *
   * @return [-DBL_MAX, DBL_MAX]
   */
  static Interval maxRepresentableReals();

  /**
   * @brief Factory method to build the set of machine representatble
   * non-negative reals.
   *
   * @return [0, DBL_MAX]
   */
  static Interval nonNegativeReals();

  /**
   * @brief Factory method to build the set of machine representatble
   * non-positive reals.
   *
   * @return [-DBL_MAX, 0]
   */
  static Interval nonPositiveReals();

  /**
   * @brief Constructor: initialize an empty interval with members set to NaN.
   */
  Interval();

  /**
   * @brief Constructor: specify exact interval bounds.
   *
   * If the interval is initialized with invalid bounds, then the bounds are set
   * to NaN.
   *
   * @param minimum The minimum bound.
   * @param maximum The maximum bound.
   */
  Interval(const double minimum, const double maximum);

 protected:
  /** @brief Scalar interval minimum bounds: min, max. */
  std::tuple<double, double> bounds_;

  /** @brief Interval constructor overload. */
  Interval(const std::tuple<double, double>& bounds);

  /** @brief What the bounds would be if the intervals were intersected. */
  static std::tuple<double, double> get_intersection_bounds(
      const Interval& interval1, const Interval& interval2);

  /** @brief Verify that the bounds are valid for an interval. */
  static bool bounds_valid(const std::tuple<double, double>& bounds);
};  // class Interval

/** @brief Define interval addition as the addition of interval bounds. */
Interval operator+(const Interval& interval1, const Interval& interval2);

/**
 * @brief Define scalar addition as the addition of a scalar to the bounds.
 * @{
 */
Interval operator+(const Interval& interval, const double scalar);
Interval operator-(const Interval& interval, const double scalar);
/** @} */
}  // namespace maeve_automation_core
