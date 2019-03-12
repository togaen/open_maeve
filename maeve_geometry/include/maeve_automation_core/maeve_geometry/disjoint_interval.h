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

#include "bounds_ordered_interval.h"

#include <algorithm>
#include <iostream>
#include <set>
#include <utility>
#include <vector>

namespace maeve_automation_core {
/**
 * @brief This class is a container for maintaining disjoint interval sets.
 */
template <typename T>
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
                                  const DisjointInterval& disjoint_interval) {
    if (disjoint_interval.set_.empty()) {
      return os << "{}";
    }
    os << "{";
    const auto it_sentry = --(std::end(disjoint_interval.set_));
    for (auto it = std::begin(disjoint_interval.set_); it != it_sentry; ++it) {
      os << *it << ", ";
    }
    return os << *it_sentry << "}";
  }

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
                         const DisjointInterval& disjoint_interval2) {
    const auto di1_size = DisjointInterval::size(disjoint_interval1);
    const auto di2_size = DisjointInterval::size(disjoint_interval2);
    if (di1_size != di2_size) {
      return false;
    }

    auto accumulator = true;
    auto it1 = std::begin(disjoint_interval1.set_);
    auto it2 = std::begin(disjoint_interval2.set_);
    for (; it1 != std::end(disjoint_interval1.set_); ++it1, ++it2) {
      accumulator = accumulator && (*it1 == *it2);
    }
    return accumulator;
  }

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
                         const DisjointInterval& disjoint_interval2) {
    return !(disjoint_interval1 == disjoint_interval2);
  }

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
  explicit DisjointInterval(const std::vector<Interval<T>>& intervals);

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
  static typename std::set<BoundsOrderedInterval<T>>::const_iterator insert(
      DisjointInterval& disjoint_interval, Interval<T> interval);

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
  static typename std::set<BoundsOrderedInterval<T>>::const_iterator begin(
      const DisjointInterval& disjoint_interval);

  /**
   * @brief Get an end iterator for a given disjoint interval set.
   *
   * @param disjoint_interval The disjoint interval set.
   *
   * @return The end iterator for the disjoint interval set.
   */
  static typename std::set<BoundsOrderedInterval<T>>::const_iterator end(
      const DisjointInterval& disjoint_interval);

  /**
   * @brief Get the size of the disjoint interval, i.e., the count of disjoint
   * intervals.
   *
   * @param disjoint_interval The disjoint interval set.
   *
   * @return The count of disjoint intervals in the set.
   */
  static typename std::set<BoundsOrderedInterval<T>>::size_type size(
      const DisjointInterval& disjoint_interval);

 private:
  /** @brief The container for all disjoint intervals. */
  std::set<BoundsOrderedInterval<T>> set_;
};  // class DisjointInterval

//------------------------------------------------------------------------------

template <typename T>
DisjointInterval<T>::DisjointInterval(
    const std::vector<Interval<T>>& intervals) {
  std::for_each(std::begin(intervals), std::end(intervals),
                [&](const Interval<T>& interval) {
                  DisjointInterval::insert(*this, interval);
                });
}

//------------------------------------------------------------------------------

template <typename T>
DisjointInterval<T> DisjointInterval<T>::intersect(
    const DisjointInterval& disjoint_interval1,
    const DisjointInterval& disjoint_interval2) {
  // Capture iterators.
  auto it1 = DisjointInterval::begin(disjoint_interval1);
  auto it1_end = DisjointInterval::end(disjoint_interval1);
  auto it2 = DisjointInterval::begin(disjoint_interval2);
  auto it2_end = DisjointInterval::end(disjoint_interval2);

  // Build return set.
  auto disjoint_interval = DisjointInterval();

  // Walk sets, compute intersection.
  while ((it1 != it1_end) && (it2 != it2_end)) {
    // Capture references.
    const auto& i1 = *it1;
    const auto& i2 = *it2;

    // Capture bounds.
    double min1, max1, min2, max2;
    std::tie(min1, max1) = Interval<T>::bounds(i1);
    std::tie(min2, max2) = Interval<T>::bounds(i2);

    // Compute intersection.
    const auto i = Interval<T>::intersect(i1, i2);

    // No intersection: Discard the interval that has no more intersections.
    if (Interval<T>::empty(i)) {
      const auto discard1 = (max1 <= min2);
      if (discard1) {
        ++it1;
      } else {
        ++it2;
      }

      // Walk forward.
      continue;
    }

    // Intersection: Add and discard the consumed interval.
    DisjointInterval::insert(disjoint_interval, i);
    const auto discard1 = (max1 <= max2);
    if (discard1) {
      ++it1;
    } else {
      ++it2;
    }
  }

  // Done.
  return disjoint_interval;
}

//------------------------------------------------------------------------------

template <typename T>
bool DisjointInterval<T>::contains(const DisjointInterval& disjoint_interval,
                                   const double value) {
  return std::any_of(std::begin(disjoint_interval.set_),
                     std::end(disjoint_interval.set_),
                     [&](const Interval<T>& interval) {
                       return Interval<T>::contains(interval, value);
                     });
}

//------------------------------------------------------------------------------

template <typename T>
typename std::set<BoundsOrderedInterval<T>>::const_iterator
DisjointInterval<T>::insert(DisjointInterval& disjoint_interval,
                            Interval<T> interval) {
  // Don't insert empty intervals.
  if (Interval<T>::empty(interval)) {
    return std::end(disjoint_interval.set_);
  }

  // If set is empty, no merging is possible, just insert. Done.
  if (disjoint_interval.set_.empty()) {
    return disjoint_interval.set_.insert(std::end(disjoint_interval.set_),
                                         interval);
  }

  // Find first element greater than or equal to 'interval'.
  auto it_hint = disjoint_interval.set_.lower_bound(interval);

  // If the set contains an element equivalent to 'interval', then the iterator
  // will point to that element. In this case, 'interval' contains no extra
  // information, so it won't merge with any existing intervals. Done.
  if ((it_hint != std::end(disjoint_interval.set_) && (*it_hint == interval))) {
    return it_hint;
  }

  // Attempt merging of previous intervals.
  {
    auto it_reverse = it_hint;
    while (it_reverse != std::begin(disjoint_interval.set_)) {
      // Back up the iterator.
      --it_reverse;

      // Attempt a merge.
      if (const auto i = Interval<T>::merge(*it_reverse, interval)) {
        interval = *i;
      } else {
        // If the merge was not successful, no other intervals are disjoint.
        // Done.
        break;
      }

      // Remove the merged element.
      it_reverse = disjoint_interval.set_.erase(it_reverse);
    }
  }

  // Attempt merging of following intervals.
  {
    auto it_forward = it_hint;
    while (it_forward != std::end(disjoint_interval.set_)) {
      // Attempt a merge
      if (const auto i = Interval<T>::merge(*it_forward, interval)) {
        interval = *i;
      } else {
        // If the merge was not successful, no other intervals are disjoint.
        // Done.
        break;
      }

      // Remove the merged element and update the iterator.
      it_forward = disjoint_interval.set_.erase(it_forward);
    }

    // Update hint iterator in case the forward element has changed.
    it_hint = it_forward;
  }

  //
  // All overlapping intervals have now been merged into 'interval'. Insert
  // 'interval' into the disjoint set.
  //

  // Done.
  return disjoint_interval.set_.insert(it_hint, interval);
}

//------------------------------------------------------------------------------

template <typename T>
typename std::set<BoundsOrderedInterval<T>>::const_iterator
DisjointInterval<T>::begin(const DisjointInterval& disjoint_interval) {
  return std::begin(disjoint_interval.set_);
}

//------------------------------------------------------------------------------

template <typename T>
typename std::set<BoundsOrderedInterval<T>>::const_iterator
DisjointInterval<T>::end(const DisjointInterval& disjoint_interval) {
  return std::end(disjoint_interval.set_);
}

//------------------------------------------------------------------------------

template <typename T>
typename std::set<BoundsOrderedInterval<T>>::size_type
DisjointInterval<T>::size(const DisjointInterval& disjoint_interval) {
  return disjoint_interval.set_.size();
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
