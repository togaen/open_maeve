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

#include "interval.h"

namespace maeve_core {
/**
 * @brief Interval view that supports ordering.
 *
 * @note For sorting, these operators sort first based on the min interval
 * bound, then the max.
 */
template <typename T>
class BoundsOrderedInterval final : public Interval<T> {
 public:
  /** @brief Constructor given unordered interval. */
  BoundsOrderedInterval(const Interval<T>& interval);

  /** @name Comparison operations
   *
   * Comparison operations for BoundsOrderedInterval types. For sorting, these
   * operators sort first based on the min interval bound, then the max.
   *
   * @note Like comparisons of NaN, invalid intervals always compare false.
   *
   * @{
   */

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
  friend bool operator<(const BoundsOrderedInterval& interval1,
                        const BoundsOrderedInterval& interval2) {
    // Capture properties.
    const auto min_eq = (BoundsOrderedInterval::min(interval1) ==
                         BoundsOrderedInterval::min(interval2));
    const auto min_lt = (BoundsOrderedInterval::min(interval1) <
                         BoundsOrderedInterval::min(interval2));
    const auto max_eq = (BoundsOrderedInterval::max(interval1) ==
                         BoundsOrderedInterval::max(interval2));
    const auto max_lt = (BoundsOrderedInterval::max(interval1) <
                         BoundsOrderedInterval::max(interval2));

    //
    // The following test all cases for ordering.
    //

    if (!BoundsOrderedInterval::can_be_ordered(interval1, interval2)) {
      return false;
    }

    //
    // From here, both intervals exhibit ordering.
    //

    if (min_lt) {
      return true;
    }

    //
    // From here, min bound of interval 1 is >= min bound of interval2.
    //

    if (!min_eq) {
      return false;
    }

    //
    // From here, min bound of interval 1 is == min bound of interval 2.
    //

    if (max_lt) {
      return true;
    }

    //
    // From here, min bound of interval 1 is == min bound of interval 2, and max
    // bound of interval 1 is >= max bound of interval 2.
    //

    return false;
  }

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
  friend bool operator<=(const BoundsOrderedInterval& interval1,
                         const BoundsOrderedInterval& interval2) {
    return (BoundsOrderedInterval::can_be_ordered(interval1, interval2) &&
            !(interval1 > interval2));
  }

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
  friend bool operator>(const BoundsOrderedInterval& interval1,
                        const BoundsOrderedInterval& interval2) {
    // Capture properties.
    const auto min_eq = (BoundsOrderedInterval::min(interval1) ==
                         BoundsOrderedInterval::min(interval2));
    const auto min_gt = (BoundsOrderedInterval::min(interval1) >
                         BoundsOrderedInterval::min(interval2));
    const auto max_eq = (BoundsOrderedInterval::max(interval1) ==
                         BoundsOrderedInterval::max(interval2));
    const auto max_gt = (BoundsOrderedInterval::max(interval1) >
                         BoundsOrderedInterval::max(interval2));

    //
    // The following test all cases for ordering.
    //

    if (!BoundsOrderedInterval::can_be_ordered(interval1, interval2)) {
      return false;
    }

    //
    // From here, both intervals exhibit ordering.
    //

    if (min_gt) {
      return true;
    }

    //
    // From here, min bound of interval 1 is <= min bound of interval2.
    //

    if (!min_eq) {
      return false;
    }

    //
    // From here, min bound of interval 1 is == min bound of interval 2.
    //

    if (max_gt) {
      return true;
    }

    //
    // From here, min bound of interval 1 is == min bound of interval 2, and max
    // bound of interval 1 is <= max bound of interval 2.
    //

    return false;
  }

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
  friend bool operator>=(const BoundsOrderedInterval& interval1,
                         const BoundsOrderedInterval& interval2) {
    return (BoundsOrderedInterval::can_be_ordered(interval1, interval2) &&
            !(interval1 < interval2));
  }

  /** }@ */

 private:
  /** @brief Two intervals can be ordered iff they are both non-emtpy. */
  static bool can_be_ordered(const BoundsOrderedInterval& interval1,
                             const BoundsOrderedInterval& interval2);
};  // class BoundsOrderedInterval

//------------------------------------------------------------------------------

template <typename T>
BoundsOrderedInterval<T>::BoundsOrderedInterval(const Interval<T>& interval)
    : Interval<T>(interval) {}

//------------------------------------------------------------------------------

template <typename T>
bool BoundsOrderedInterval<T>::can_be_ordered(
    const BoundsOrderedInterval& interval1,
    const BoundsOrderedInterval& interval2) {
  return (!BoundsOrderedInterval::empty(interval1) &&
          !BoundsOrderedInterval::empty(interval2));
}

//------------------------------------------------------------------------------

}  // namespace maeve_core
