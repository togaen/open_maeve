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
#include "maeve_automation_core/maeve_geometry/bounds_ordered_interval.h"

namespace maeve_automation_core {

//------------------------------------------------------------------------------

BoundsOrderedInterval::BoundsOrderedInterval(const Interval& interval)
    : Interval(interval) {}

//------------------------------------------------------------------------------

bool operator<(const BoundsOrderedInterval& interval1,
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
  const auto can_be_ordered =
      (BoundsOrderedInterval::exhibitsOrdering(interval1) &&
       BoundsOrderedInterval::exhibitsOrdering(interval2));

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

//------------------------------------------------------------------------------

bool operator>=(const BoundsOrderedInterval& interval1,
                const BoundsOrderedInterval& interval2) {
  const auto can_be_ordered =
      (BoundsOrderedInterval::exhibitsOrdering(interval1) &&
       BoundsOrderedInterval::exhibitsOrdering(interval2));
  return can_be_ordered && !(interval1 < interval2);
}

//------------------------------------------------------------------------------

bool operator>(const BoundsOrderedInterval& interval1,
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
  const auto can_be_ordered =
      (BoundsOrderedInterval::exhibitsOrdering(interval1) &&
       BoundsOrderedInterval::exhibitsOrdering(interval2));

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

//------------------------------------------------------------------------------

bool operator<=(const BoundsOrderedInterval& interval1,
                const BoundsOrderedInterval& interval2) {
  const auto can_be_ordered =
      (BoundsOrderedInterval::exhibitsOrdering(interval1) &&
       BoundsOrderedInterval::exhibitsOrdering(interval2));
  return can_be_ordered && !(interval1 > interval2);
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
