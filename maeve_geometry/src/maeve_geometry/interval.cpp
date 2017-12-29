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

double Interval::min(const Interval& interval) {
  return std::get<0>(interval.bounds_);
}

double Interval::max(const Interval& interval) {
  return std::get<1>(interval.bounds_);
}

bool Interval::empty(const Interval& interval) { return interval.empty_; }

Interval::Interval()
    : bounds_(std::move(std::make_tuple(NaN, NaN))), empty_(true) {}

Interval::Interval(const double minimum, const double maximum)
    : bounds_(std::move(std::make_tuple(minimum, maximum))), empty_(false) {
  if (!Interval::valid(*this)) {
    bounds_ = std::move(std::make_tuple(NaN, NaN));
  }
}

bool Interval::contains(const Interval& interval, const double value) {
  return (Interval::valid(interval) && !Interval::empty(interval) &&
          (value >= Interval::min(interval)) &&
          (value <= Interval::max(interval)));
}

double Interval::length(const Interval& interval) {
  return Interval::empty(interval)
             ? NaN
             : (Interval::max(interval) - Interval::min(interval));
}

bool Interval::valid(const Interval& interval) {
  return (Interval::empty(interval) ||
          (Interval::min(interval) <= Interval::max(interval)));
}

Interval Interval::convexHull(const Interval& interval1,
                              const Interval& interval2) {
  auto i =
      Interval(std::min(Interval::min(interval1), Interval::min(interval2)),
               std::max(Interval::max(interval1), Interval::max(interval2)));
  i.empty_ = Interval::empty(interval1) && Interval::empty(interval2);
  return i;
}

Interval Interval::intersection(const Interval& interval1,
                                const Interval& interval2) {
  auto i =
      Interval(std::max(Interval::min(interval1), Interval::min(interval2)),
               std::min(Interval::max(interval1), Interval::max(interval2)));
  i.empty_ = Interval::empty(interval1) || Interval::empty(interval2);
  return i;
}

bool Interval::exhibitsOrdering(const Interval& interval) {
  return !Interval::empty(interval) && Interval::valid(interval);
}

bool operator==(const Interval& interval1, const Interval& interval2) {
  // Capture properties.
  const auto min_eq = (Interval::min(interval1) == Interval::min(interval2));
  const auto max_eq = (Interval::max(interval1) == Interval::max(interval2));
  const auto empty = (Interval::empty(interval1) && Interval::empty(interval2));

  // Compute equality.
  return (empty || (min_eq && max_eq));
}

bool operator!=(const Interval& interval1, const Interval& interval2) {
  return !(interval1 == interval2);
}

bool operator<(const Interval& interval1, const Interval& interval2) {
  // Capture properties.
  const auto min_eq = (Interval::min(interval1) == Interval::min(interval2));
  const auto min_lt = (Interval::min(interval1) < Interval::min(interval2));
  const auto max_eq = (Interval::max(interval1) == Interval::max(interval2));
  const auto max_lt = (Interval::max(interval1) < Interval::max(interval2));
  const auto can_be_ordered = (Interval::exhibitsOrdering(interval1) &&
                               Interval::exhibitsOrdering(interval2));

  //
  // The following test all cases for ordering.
  //

  if (can_be_ordered) {
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

bool operator>=(const Interval& interval1, const Interval& interval2) {
  const auto can_be_ordered = (Interval::exhibitsOrdering(interval1) &&
                               Interval::exhibitsOrdering(interval2));
  return can_be_ordered && !(interval1 < interval2);
}

bool operator>(const Interval& interval1, const Interval& interval2) {
  // Capture properties.
  const auto min_eq = (Interval::min(interval1) == Interval::min(interval2));
  const auto min_gt = (Interval::min(interval1) > Interval::min(interval2));
  const auto max_eq = (Interval::max(interval1) == Interval::max(interval2));
  const auto max_gt = (Interval::max(interval1) > Interval::max(interval2));
  const auto can_be_ordered = (Interval::exhibitsOrdering(interval1) &&
                               Interval::exhibitsOrdering(interval2));

  //
  // The following test all cases for ordering.
  //

  if (can_be_ordered) {
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

bool operator<=(const Interval& interval1, const Interval& interval2) {
  const auto can_be_ordered = (Interval::exhibitsOrdering(interval1) &&
                               Interval::exhibitsOrdering(interval2));
  return can_be_ordered && !(interval1 > interval2);
}

}  // namespace maeve_automation_core
