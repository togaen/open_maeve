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
#include <tuple>

namespace maeve_automation_core {
/**
 * @brief Data structure to contain scalar interval bounds.
 */
class Interval {
 public:
  /** @name Comparison operations
   *
   * Comparison operations for Interval types. For sorting, these operators sort
   * first based on the min interval bound, then the max.
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

  /**
   * @brief Compute whether an interval is strictly less than another.
   *
   * An interval is strictly less than another if its minimum bound is strictly
   * less than the other's, or, if the min bound is equal, its max bound is
   * strictly less than the other's.
   *
   * @param interval1 The first interval to compare.
   * @param interval2 The second interval to compare.
   *
   * @return True if 'interval1' is strictly less than 'interval2'; otherwise
   * false.
   */
  friend bool operator<(const Interval& interval1, const Interval& interval2);

  /**
   * @brief Compute whether an interval is less than or equal to another.
   *
   * An interval is less than or equal to another if it is not strictly greater
   * than the other.
   *
   * @param interval1 The first interval to compare.
   * @param interval2 The second interval to compare.
   *
   * @return True if 'interval1' is less than or equal to 'interval2'; otherwise
   * false.
   */
  friend bool operator<=(const Interval& interval1, const Interval& interval2);

  /**
   * @brief Compute whether an interval is strictly greater than another.
   *
   * An interval is strictly greater than another if its minimum bound is
   * strictly greater than the other's, or, if the min bound is equal, its max
   * bound is strictly greater than the other's.
   *
   * @param interval1 The first interval to compare.
   * @param interval2 The second interval to compare.
   *
   * @return True if 'interval1' is strictly greater than 'interval2'; otherwise
   * false.
   */
  friend bool operator>(const Interval& interval1, const Interval& interval2);

  /**
   * @brief Compute whether an interval is greater than or equal to another.
   *
   * And interval is greater than or equal to another if it is not strictly less
   * than the other.
   *
   * @param interval1 The first interval to compare.
   * @param interval2 The second interval to compare.
   *
   * @return True if 'interval1' is greater than or equal to 'interval2';
   * otherwise false.
   */
  friend bool operator>=(const Interval& interval1, const Interval& interval2);
  /**
   * }@
   * */

  /**
   * @brief Stream overload for Interval types.
   *
   * @note Output precision is fixed inside the function definition.
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
   * @param interval
   *
   * @return The length of the interval.
   */
  static double length(const Interval& interval);

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
   * @brief Whether the interval is valid.
   *
   * @note An interval with NaN in the bounds is only valid if it is empty.
   *
   * @param interval The interval to test for validity.
   *
   * @return True if empty or min <= max; otherwise false.
   */
  static bool valid(const Interval& interval);

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
   * This method computes and returns a convex hull of two intervals iff they
   * intersect. If the two intervals do not intersect, there is no interval that
   * exactly contains the input intervals, so an invalid interval is returned.
   *
   * @param interval1 The first interval to merge.
   * @param interval2 The second interval to merge.
   *
   * @return The merged interval.
   */
  static Interval merge(const Interval& interval1, const Interval& interval2);

  /**
   * @brief Factory method for building an interval representing the affinely
   * extended reals.
   *
   * @return [-\infty, +\infty]
   */
  static Interval affinelyExtendedReals();

  /**
   * @brief Factory method for build an interval representing the maximum range
   * of representable reals.
   *
   * @return [-DBL_MAX, DBL_MAX]
   */
  static Interval maxRepresentableReals();

  /**
   * @brief Constructor: initialize and empty interval with members set to NaN.
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

 private:
  /** @brief Scalar interval minimum bounds: min, max. */
  std::tuple<double, double> bounds_;
  /** @brief Whether the interval is empty or not. */
  bool empty_;

  /**
   * @brief Construct and return an invalid interval.
   *
   * @return The invalid interval.
   */
  static Interval buildInvalid();

  /**
   * @brief Test for whether a given interval can be ordered.
   *
   * Intervals exhibit ordering iff they are neither empty nor invalid.
   *
   * @param interval The interval to test for ordering property.
   *
   * @return True if the interval can be ordered; otherwise false.
   */

  static bool exhibitsOrdering(const Interval& interval);
};  // class Interval
}  // namespace maeve_automation_core
