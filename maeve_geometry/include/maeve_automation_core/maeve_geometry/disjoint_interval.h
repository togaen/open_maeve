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

#include <set>
#include <utility>

#include "maeve_automation_core/maeve_geometry/interval.h"

namespace maeve_automation_core {
class DisjointInterval {
 public:
  /**
   * @brief Insert an interval into a disjoint interval set.
   *
   * Insertion does two things: first, it places the interval in the set;
   * second, it merges the interval with preceding or following intervals if
   * there is any overlap. In this way, the set of disjoint intervals contains
   * exactly the set of points contained by all inserted intervals, and each
   * individual interval is disjoint from all others.
   *
   * @note Invalid intervals will not be inserted into the set.
   *
   * @param disjoint_interval The target disjoint interval.
   * @param interval The interval to be inserted.
   *
   * @return See std::set<T>::insert(std::set<T>::value_type&& val)
   */
  static std::pair<std::set<Interval>::iterator, bool> insert(
      DisjointInterval& disjoint_interval, Interval&& interval);

 private:
  /** @brief The container for all disjoint intervals. */ 
  std::set<Interval> set_;
};  // class DisjointInterval
}  // namespace maeve_automation_core
