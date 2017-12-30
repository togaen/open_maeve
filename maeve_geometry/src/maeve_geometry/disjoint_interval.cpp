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

#include <cassert>

namespace maeve_automation_core {
std::pair<std::set<Interval>::iterator, bool> DisjointInterval::insert(
    DisjointInterval& disjoint_interval, Interval interval) {
  // Don't insert invalid or empty intervals.
  if (!Interval::valid(interval) || Interval::empty(interval)) {
    return std::make_pair(std::end(disjoint_interval.set_), false);
  }

  // If set is empty, no merging is possible. Done.
  if (disjoint_interval.set_.empty()) {
    return disjoint_interval.set_.insert(std::move(interval));
  }

  // Find first element greater than or equal to 'interval'.
  const auto it = disjoint_interval.set_.lower_bound(interval);

  // If the set contains an element equivalent to 'interval', then the iterator
  // will point to that element. In this case, 'interval' contains no extra
  // information, so it won't merge with any existing intervals. Done.
  if (*it == interval) {
    return std::make_pair(it, false);
  }

  // Attempt merging of previous intervals.
  {
    auto it_reverse = it;
    while (it_reverse != std::begin(disjoint_interval.set_)) {
      // Back up the iterator.
      --it_reverse;

      // Attempt a merge.
      const auto i = Interval::merge(*it_reverse, interval);

      // If the merge was not successful, all other previous intervals will be
      // disjoint. Done.
      if (!Interval::valid(i)) {
        break;
      }

      //
      // The merge was successful: remove the merged interval and update the
      // insertion interval.
      //

      // Move the tracking interator back to keep it valid.
      auto it_remove = it_reverse;
      if (it_reverse != std::begin(disjoint_interval.set_)) {
        --it_reverse;
      }

      // Remove the merged element.
      const auto it_tmp = disjoint_interval.set_.erase(it_remove);

      // Update the inerstion interval.
      interval = i;
    }
  }

  // Attempt merging of following intervals.
  {
    auto it_forward = it;
    while (it_forward != std::end(disjoint_interval.set_)) {
      // Attempt a merge
      const auto i = Interval::merge(*it_forward, interval);

      // If the merge was not successful, all other forward intervals will be
      // disjoint. Done.
      if (!Interval::valid(i)) {
        break;
      }

      //
      // The merge was successful: remove the merged interval and update the
      // insertion interval.
      //

      // Remove the merged element and update the iterator.
      it_forward = disjoint_interval.set_.erase(it_forward);

      // Update the insertion interval.
      interval = i;
    }
  }

  //
  // All overlapping intervals have now been merged into 'interval'. Insert
  // 'interval' into the disjoint set.
  //

  // Done.
  return disjoint_interval.set_.insert(std::move(interval));
}

std::set<Interval>::const_iterator DisjointInterval::begin(
    const DisjointInterval& disjoint_interval) {
  return std::begin(disjoint_interval.set_);
}

std::set<Interval>::const_iterator DisjointInterval::end(
    const DisjointInterval& disjoint_interval) {
  return std::end(disjoint_interval.set_);
}

}  // namespace maeve_automation_core
