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
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <boost/io/ios_state.hpp>
#include <boost/optional.hpp>

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

const std::tuple<double, double>& Interval::bounds(const Interval& interval) {
  return interval.bounds_;
}

bool Interval::zeroLength(const Interval& interval) {
  const auto is_not_empty = !Interval::empty(interval);
  const auto length_is_zero =
      (Interval::min(interval) == Interval::max(interval));
  return (is_not_empty && length_is_zero);
}

bool Interval::empty(const Interval& interval) {
  return (std::isnan(Interval::min(interval)) &&
          std::isnan(Interval::max(interval)));
}

Interval Interval::affinelyExtendedReals() {
  return Interval(INFIMUM, SUPREMUM);
}

Interval Interval::maxRepresentableReals() { return Interval(MIN, MAX); }

Interval Interval::nonNegativeReals() { return Interval(0.0, MAX); }

Interval Interval::nonPositiveReals() { return Interval(MIN, 0.0); }

Interval::Interval() : bounds_(std::make_tuple(NaN, NaN)) {}

Interval::Interval(const double minimum, const double maximum)
    : bounds_(std::make_tuple(minimum, maximum)) {
  if (!Interval::bounds_valid(bounds_)) {
    std::stringstream ss;
    ss << "Attempted to construct an invalid interval: " << *this;
    throw std::runtime_error(ss.str());
  }
}

Interval::Interval(const std::tuple<double, double>& bounds)
    : Interval(std::get<0>(bounds), std::get<1>(bounds)) {}

bool Interval::bounds_valid(const std::tuple<double, double>& bounds) {
  const auto min = std::get<0>(bounds);
  const auto max = std::get<1>(bounds);
  const auto is_empty = (std::isnan(min) && std::isnan(max));
  const auto is_ordered = (min <= max);
  return (is_empty || is_ordered);
}

Interval operator+(const Interval& interval1, const Interval& interval2) {
  const auto min = (Interval::min(interval1) + Interval::min(interval2));
  const auto max = (Interval::max(interval1) + Interval::max(interval2));
  return Interval(min, max);
}

bool Interval::isSubsetEq(const Interval& interval1,
                          const Interval& interval2) {
  const auto lower_contained =
      (Interval::min(interval1) >= Interval::min(interval2));
  const auto upper_contained =
      (Interval::max(interval1) <= Interval::max(interval2));
  return (lower_contained && upper_contained);
}

bool Interval::contains(const Interval& interval, const double value) {
  return ((value >= Interval::min(interval)) &&
          (value <= Interval::max(interval)));
}

double Interval::length(const Interval& interval) {
  return (Interval::max(interval) - Interval::min(interval));
}

Interval Interval::convexHull(const Interval& interval1,
                              const Interval& interval2) {
  // If either interval is empty, the hull is trivially the other interval.
  if (Interval::empty(interval1)) {
    return interval2;
  }
  if (Interval::empty(interval2)) {
    return interval1;
  }

  // Otherwise, the hull contains both.
  return Interval(std::min(Interval::min(interval1), Interval::min(interval2)),
                  std::max(Interval::max(interval1), Interval::max(interval2)));
}

boost::optional<Interval> Interval::merge(const Interval& interval1,
                                          const Interval& interval2) {
  // If the intervals do not overlap, they cannot be merged.
  if (!Interval::overlap(interval1, interval2)) {
    return boost::none;
  }

  // Done.
  return Interval::convexHull(interval1, interval2);
}

Interval Interval::intersect(const Interval& interval1,
                             const Interval& interval2) {
  // Construct intersection assuming an intersection exists.
  try {
    return Interval(Interval::get_intersection_bounds(interval1, interval2));
  } catch (...) {
  }

  // Otherwise, the intersection is empty.
  return Interval();
}

std::tuple<double, double> Interval::get_intersection_bounds(
    const Interval& interval1, const Interval& interval2) {
  const auto either_empty =
      (Interval::empty(interval1) || Interval::empty(interval2));

  const auto min = std::max(Interval::min(interval1), Interval::min(interval2));
  const auto max = std::min(Interval::max(interval1), Interval::max(interval2));

  return (either_empty ? std::make_tuple(NaN, NaN) : std::make_tuple(min, max));
}

bool Interval::overlap(const Interval& interval1, const Interval& interval2) {
  const auto bounds = Interval::get_intersection_bounds(interval1, interval2);
  return Interval::bounds_valid(bounds);
}

double Interval::projectToInterval(const Interval& interval, const double val) {
  if (Interval::contains(interval, val)) {
    return val;
  }
  return ((val < Interval::min(interval)) ? Interval::min(interval)
                                          : Interval::max(interval));
}

double Interval::scale_to_interval(const Interval& interval, const double val) {
  if (std::isnan(val) || Interval::empty(interval)) {
    return NaN;
  }

  const auto l = Interval::length(interval);
  const auto offset = (val - Interval::min(interval));

  return (offset / l);
}

bool Interval::exhibitsOrdering(const Interval& interval) {
  return !Interval::empty(interval);
}

bool operator==(const Interval& interval1, const Interval& interval2) {
  // Capture properties.
  const auto min_eq = (Interval::min(interval1) == Interval::min(interval2));
  const auto max_eq = (Interval::max(interval1) == Interval::max(interval2));
  const auto both_empty =
      (Interval::empty(interval1) && Interval::empty(interval2));

  // Compute equality: for empty intervals min_eq and max_eq are both 'false'
  // because the bounds are all NaN.
  return (both_empty || (min_eq && max_eq));
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

  if (!can_be_ordered) {
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

  if (!can_be_ordered) {
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

std::ostream& operator<<(std::ostream& os, const Interval& interval) {
  static const auto PRECISION = 5;

  // Restore stream state on exit.
  boost::io::ios_all_saver guard(os);

  // Temporarily set desired flags and precision.
  os.setf(std::ios::fixed, std::ios::floatfield);
  os.precision(PRECISION);

  // Do stream output.
  if (Interval::empty(interval)) {
    return os << "{}";
  }

  return os << "{\"min\": " << Interval::min(interval)
            << ", \"max\": " << Interval::max(interval) << "}";
}
}  // namespace maeve_automation_core
