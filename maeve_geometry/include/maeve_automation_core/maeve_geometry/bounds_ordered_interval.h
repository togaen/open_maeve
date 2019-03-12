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

namespace maeve_automation_core {
/**
 * @brief Interval view that supports ordering.
 *
 * @note For sorting, these operators sort first based on the min interval
 * bound, then the max.
 */
class BoundsOrderedInterval final : public Interval {
 public:
  /** @brief Constructor given unordered interval. */
  BoundsOrderedInterval(const Interval& interval);

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
                        const BoundsOrderedInterval& interval2);

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
                         const BoundsOrderedInterval& interval2);

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
                        const BoundsOrderedInterval& interval2);

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
                         const BoundsOrderedInterval& interval2);

  /** }@ */

 private:
  /** @brief Two intervals can be ordered iff they are both non-emtpy. */
  static bool can_be_ordered(const BoundsOrderedInterval& interval1,
                             const BoundsOrderedInterval& interval2);
};  // class BoundsOrderedInterval

}  // namespace maeve_automation_core
