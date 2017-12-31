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
#include "maeve_automation_core/maeve_geometry/disjoint_interval.h"

#include <algorithm>

namespace maeve_automation_core {
DisjointInterval::DisjointInterval(const std::vector<Interval>& intervals) {
  std::for_each(std::begin(intervals), std::end(intervals),
                [&](const Interval& interval) {
                  DisjointInterval::insert(*this, interval);
                });
}

std::set<Interval>::const_iterator DisjointInterval::insert(
    DisjointInterval& disjoint_interval, Interval interval) {
  // Don't insert invalid or empty intervals.
  if (!Interval::valid(interval) || Interval::empty(interval)) {
    return std::end(disjoint_interval.set_);
  }

  // If set is empty, no merging is possible, just insert. Done.
  if (disjoint_interval.set_.empty()) {
    return disjoint_interval.set_.insert(std::end(disjoint_interval.set_),
                                         std::move(interval));
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
      const auto i = Interval::merge(*it_reverse, interval);

      // If the merge was not successful, no other intervals are disjoint. Done.
      if (!Interval::valid(i)) {
        break;
      }

      // Update the inerstion interval.
      interval = i;

      // Remove the merged element.
      it_reverse = disjoint_interval.set_.erase(it_reverse);
    }
  }

  // Attempt merging of following intervals.
  {
    auto it_forward = it_hint;
    while (it_forward != std::end(disjoint_interval.set_)) {
      // Attempt a merge
      const auto i = Interval::merge(*it_forward, interval);

      // If the merge was not successful, no other intervals are disjoint. Done.
      if (!Interval::valid(i)) {
        break;
      }

      // Update the insertion interval.
      interval = i;

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
  return disjoint_interval.set_.insert(it_hint, std::move(interval));
}

std::set<Interval>::const_iterator DisjointInterval::begin(
    const DisjointInterval& disjoint_interval) {
  return std::begin(disjoint_interval.set_);
}

std::set<Interval>::const_iterator DisjointInterval::end(
    const DisjointInterval& disjoint_interval) {
  return std::end(disjoint_interval.set_);
}

std::set<Interval>::size_type DisjointInterval::size(
    const DisjointInterval& disjoint_interval) {
  return disjoint_interval.set_.size();
}

std::ostream& operator<<(std::ostream& os,
                         const DisjointInterval& disjoint_interval) {
  if (disjoint_interval.set_.empty()) {
    return os << "{(empty)}";
  }
  os << "{";
  const auto it_sentry = --(std::end(disjoint_interval.set_));
  for (auto it = std::begin(disjoint_interval.set_); it != it_sentry; ++it) {
    os << *it << ", ";
  }
  return os << *it_sentry << "}";
}

bool operator==(const DisjointInterval& disjoint_interval1,
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

bool operator!=(const DisjointInterval& disjoint_interval1,
                const DisjointInterval& disjoint_interval2) {
  return !(disjoint_interval1 == disjoint_interval2);
}

}  // namespace maeve_automation_core
