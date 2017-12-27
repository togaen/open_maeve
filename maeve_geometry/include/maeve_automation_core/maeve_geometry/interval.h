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

namespace maeve_automation_core {
/**
 * @brief Data structure to contain scalar interval bounds.
 */
class Interval {
 public:
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
   * @brief Return the length of the interval.
   *
   * @note For empty or invalid intervals, NaN is returned.
   *
   * @param interval
   *
   * @return The length of the interval.
   */
  static double length(const Interval& interval);

  /**
   * @brief Whether the interval is empty or not.
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
   * @return True if min <= max; otherwise false.
   */
  static bool valid(const Interval& interval);

  /**
   * @brief Construct and return an empty interval.
   *
   * @return The empty interval.
   */
  static Interval buildEmpty();

  /**
   * @brief Construct an interval with specified bounds.
   *
   * If the given bounds are invalid, a non-empty, invalid interval is returned
   * whose bounds are set to NaN.
   *
   * @param min The minimum interval bound.
   * @param max The maximum interval bound.
   *
   * @return The constructed interval.
   */
  static Interval build(const double min, const double max);

  /**
   * @brief Compute the intersection of two intervals as a new interval.
   *
   * @param interval1 The first interval.
   * @param interval2 The second interval.
   *
   * @return The newly constructed interval intersection.
   */
  static Interval intersection(const Interval& interval1,
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

 private:
  /** @brief Scalar interval minimum bound. */
  double min_;
  /** @brief Scalar interval maximum bound. */
  double max_;
  /** @brief Whether the interval is empty or not. */
  bool empty_;

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
};  // class Interval
}  // namespace maeve_automation_core
