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
#include "maeve_automation_core/maeve_geometry/interval.h"

#include <algorithm>
#include <limits>

namespace {
const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

namespace maeve_automation_core {

double Interval::min() const { return min_; }

double Interval::max() const { return max_; }

bool Interval::empty() const { return empty_; }

Interval::Interval() : min_(NaN), max_(NaN), empty_(true) {}

Interval::Interval(const double minimum, const double maximum)
    : min_(minimum), max_(maximum), empty_(false) {
  if (!Interval::valid(*this)) {
    min_ = NaN;
    max_ = NaN;
  }
}

Interval Interval::buildEmpty() { return Interval(); }

Interval Interval::build(const double min, const double max) {
  return Interval(min, max);
}

bool Interval::valid(const Interval& interval) {
  return (interval.empty() || (interval.min() <= interval.max()));
}

Interval Interval::convexHull(const Interval& interval1,
                              const Interval& interval2) {
  return build(std::min(interval1.min(), interval2.min()),
               std::max(interval1.max(), interval2.max()));
}

Interval Interval::intersection(const Interval& interval1,
                                const Interval& interval2) {
  return build(std::max(interval1.min(), interval2.min()),
               std::min(interval1.max(), interval2.max()));
}

}  // namespace maeve_automation_core
