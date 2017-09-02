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

namespace maeve_automation_core {
/**
 * @brief A container for shape parameters.
 */
struct ShapeParameters {
  /** @brief The minimum of the closed interval constraint range. */
  double range_min;
  /** @brief The maximum of the closed interval constraint range. */
  double range_max;
  /** @brief The mid-point of [range_min, range_max]. */
  double range_mid;
  /** @brief The alpha shape parameter. */
  double alpha;
  /** @brief The beta shape parameter. */
  double beta;
  /**
   * @brief Constructor: initialize to invalid values.
   */
  ShapeParameters();
  /**
   * @brief Constructor: explicit initialization.
   *
   * @param r_min The minimum of the range being mapped onto.
   * @param r_max The maximum of the range being mapped onto.
   * @param alpha The alpha shape.
   * @param beta  The beta shape.
   */
  ShapeParameters(const double r_min, const double r_max, const double alpha,
                  const double beta);
  /**
   * @brief Convenience method to compute range_mid member.
   *
   * This method assumes range_min and range_max have both been set.
   */
  void computeMidPoint();
  /**
   * @brief Sanity check whether parameters make sense.
   *
   * @param check_range_order Whether to check that r_min <= r_max.
   *
   * @return True if parameters pass sanity check; otherwise false.
   */
  __attribute__((warn_unused_result)) bool valid(
      const bool check_range_order = true) const;
};  // struct ShapeParameters

/**
 * @brief Overload output stream operator for shape parameter set.
 *
 * @param o The output stream.
 * @param sp The shape parameters object.
 *
 * @return The stream.
 */
std::ostream& operator<<(std::ostream& o, const ShapeParameters& sp);

}  // namespace maeve_automation_core
