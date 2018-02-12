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
#include <set>
#include <utility>
#include <vector>

#include "maeve_automation_core/maeve_geometry/interval.h"

namespace maeve_automation_core {
/**
 * @brief This class is a container for maintaining disjoint interval sets.
 */
class DisjointInterval {
 public:
  /**
   * @brief Stream overload for DisjointInterval types.
   *
   * @note The serialization is JSON compatible.
   *
   * @param os The output stream.
   * @param disjoint_interval The disjoint interval to serialize.
   *
   * @return The output stream with the serialized disjoint interval.
   */
  friend std::ostream& operator<<(std::ostream& os,
                                  const DisjointInterval& disjoint_interval);

  /**
   * @brief Compute equality.
   *
   * Two disjoint interval sets compare equal iff all member intervals compare
   * equal.
   *
   * @param disjoint_interval1 The first disjoint interval set to compare.
   * @param disjoint_interval2 The second disjoint interval set to compare.
   *
   * @return True if the disjoint interval sets compare equal; otherwise false.
   */
  friend bool operator==(const DisjointInterval& disjoint_interval1,
                         const DisjointInterval& disjoint_interval2);

  /**
   * @brief Compute inequality.
   *
   * Two disjoint interval sets compare equal iff the disjoint intervals do not
   * compare equal.
   *
   * @param disjoint_interval1 The first disjoint interval set to compare.
   * @param disjoint_interval2 The second disjoint interval set to compare.
   *
   * @return True if the disjoint interval sets compare not equal; otherwise
   * false.
   */
  friend bool operator!=(const DisjointInterval& disjoint_interval1,
                         const DisjointInterval& disjoint_interval2);

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
   * @brief Constructor: build empty disjoint interval set.
   */
  DisjointInterval() = default;

  /**
   * @brief Constructor: build a disjoint interval set from given container of
   * intervals.
   *
   * This constructor iterates over the given container calling
   * DisjointInterval::insert for each element. During construction, it is
   * useful to use the initializer syntax for std::vector, this allows
   * statements of the form: DisjointInterval({Interval(0,1), Interval(2,3)}).
   *
   * @param intervals The container of intervals to construct the set with.
   */
  explicit DisjointInterval(const std::vector<Interval>& intervals);

  /**
   * @brief Insert an interval into a disjoint interval set.
   *
   * Insertion does two things: first, it places the interval in the set;
   * second, it merges the interval with preceding or following intervals if
   * there is any overlap. In this way, the set of disjoint intervals contains
   * exactly the set of points contained by all inserted intervals, and each
   * individual interval is disjoint from all others.
   *
   * @note Invalid or empty intervals will not be inserted into the set.
   *
   * @param disjoint_interval The target disjoint interval.
   * @param interval The interval to be inserted (passed by value to give the
   * function a copy it can modify).
   *
   * @return An iterator to the inserted element, or to the element equivalent
   * to the insertion interval. If insertion fails, std::set<T>::end() is
   * returned.
   */
  static std::set<Interval>::const_iterator insert(
      DisjointInterval& disjoint_interval, Interval interval);

  /**
   * @brief Whether a given value is contained by any interval in the set.
   *
   * @param disjoint_interval The disjoint interval set.
   * @param value The value to test for containment.
   *
   * @return True if 'value' is contained by any interval in the set; otherwise
   * false;
   */
  static bool contains(const DisjointInterval& disjoint_interval,
                       const double value);

  /**
   * @brief Compute the intersection of two disjoint interval sets.
   *
   * @param disjoint_interval1 The first disjoint interval.
   * @param disjoint_interval2 The second disjoint interval.
   *
   * @return The set of disjoint intervals exactly contained by
   * 'disjoint_interval1' and 'disjoint_interval2'.
   */
  static DisjointInterval intersect(const DisjointInterval& disjoint_interval1,
                                    const DisjointInterval& disjoint_interval2);

  /**
   * @brief Get a begin iterator for a given disjoint interval set.
   *
   * @param disjoint_interval The disjoint interval set.
   *
   * @return The begin iterator for the disjoint interval.
   */
  static std::set<Interval>::const_iterator begin(
      const DisjointInterval& disjoint_interval);

  /**
   * @brief Get an end iterator for a given disjoint interval set.
   *
   * @param disjoint_interval The disjoint interval set.
   *
   * @return The end iterator for the disjoint interval set.
   */
  static std::set<Interval>::const_iterator end(
      const DisjointInterval& disjoint_interval);

  /**
   * @brief Get the size of the disjoint interval, i.e., the count of disjoint
   * intervals.
   *
   * @param disjoint_interval The disjoint interval set.
   *
   * @return The count of disjoint intervals in the set.
   */
  static std::set<Interval>::size_type size(
      const DisjointInterval& disjoint_interval);

 private:
  /** @brief The container for all disjoint intervals. */
  std::set<Interval> set_;
};  // class DisjointInterval
}  // namespace maeve_automation_core
