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

#include <array>
#include <cassert>
#include <iostream>
#include <tuple>

#include "boost/optional.hpp"

#include "maeve_automation_core/maeve_geometry/interval.h"
#include "maeve_automation_core/maeve_geometry/polynomial.h"

namespace maeve_automation_core {
/**
 * @brief This class defines the canonical form of a PST connecting trajectory.
 *
 * The canonical form of a PST connecting trajectory is
 * Parabolic-Linear-Parabolic. The trajectory begins at the first switching
 * time, changes to the linear portion at the second switching time, changes to
 * the terminal parabolic portion at the third switching time, and ends at the
 * fourth switching time. The linear portion of the trajectory is represented as
 * a parabola with a zero coefficient for the polynomial term.
 */
class PST_Connector {
 public:
  /**
   * @brief Stream overload for PST Connectors.
   *
   * @param os The output stream.
   * @param constraints The PST connector object.
   *
   * @return The output stream with the object serialized.
   */
  friend std::ostream& operator<<(std::ostream& os,
                                  const PST_Connector& connector);
  /**
  * @brief Constructor: explicitly initialize the connector.
  *
  * @note This constructor checks for validity of the arguments and throws an
  * exception if they do not meet basic necessary conditions.
  *
  * @param switching_times Trajectory switching times.
  * @param functions Trajectory functional segments.
  */
  PST_Connector(std::array<double, 4>&& switching_times,
                std::array<Polynomial, 3>&& functions);

  /**
   * @brief Get the speed at the beginning of the connector.
   *
   * @param connector The connecting trajectory.
   *
   * @return The initial speed.
   */
  static double initialSpeed(const PST_Connector& connector);

  /**
   * @brief Get the speed at the end of the connector.
   *
   * @param connector The connecting trajectory.
   *
   * @return The terminal speed.
   */
  static double terminalSpeed(const PST_Connector& connector);

  /**
   * @brief Compute and return the boundary points of a connector.
   *
   * @param connector
   *
   * @return An ordered pair of points at [min, max] domain time.
   */
  static std::tuple<Eigen::Vector2d, Eigen::Vector2d> boundaryPoints(
      const PST_Connector& connector);

  /**
    * @brief Compute an LP connector between 'p1' and 'p1'.
    *
    * The computed connector shall have a P segement that passes through 'p2'
    * with derivatives 'p2_dt' and 'p2_ddt', and an L segment through 'p1' and
    * 'p2'.
    *
    * @param p1 The initial point in PT space.
    * @param p2 The terminal point in PT space.
    * @param p2_dt Connector first derivative at 'p2'.
    * @param p2_ddt Connector second derivative.
    * @param I_dt The interval of feasible speeds.
    *
    * @return A nullable of object of either the connector or boost::none.
    */
  static boost::optional<PST_Connector> computeLP(const Eigen::Vector2d& p1,
                                                  const Eigen::Vector2d& p2,
                                                  const double p2_dt,
                                                  const double p2_ddt,
                                                  const Interval& I_dt);

  /**
   * @brief Compute a PLP connector between 'p1' and 'p1'.
   *
   * @param p1 The initial point in PT space.
   * @param p1_dt Connector first derivative at 'p1'.
   * @param p2 The terminal point in PT space.
   * @param p2_dt Connector first derivative at 'p2'.
   * @param p2_ddt Connector second derivative at 'p2'.
   * @param I_dt The interval of feasible speeds.
   *
   * @return A nullable object of either the connector or boost::none.
   */
  static boost::optional<PST_Connector> computePLP(
      const Eigen::Vector2d& p1, const double p1_dt, const Eigen::Vector2d& p2,
      const double p2_dt, const double p2_ddt, const Interval& I_dt);

  /**
   * @brief Compute a PL_0P connector between 'p1' and 'p1'.
   *
   * The PL_0P connector is a special case of the PLP connect whose linear
   * portion has zero speed.
   *
   * @param p1 The initial point in PT space.
   * @param p1_dt Connector first derivative at 'p1'.
   * @param p1_ddt Connector second deriviative at 'p1'.
   * @param p2 The terminal point in PT space.
   * @param p2_ddt Connector second derivative at 'p2'.
   * @param I_dt The interval of feasible speeds.
   *
   * @return A nullable object of either the connector or boost::none.
   */
  static boost::optional<PST_Connector> computePL_0P(
      const Eigen::Vector2d& p1, const double p1_dt, const double p1_ddt,
      const Eigen::Vector2d& p2, const double p2_ddt, const Interval& I_dt);

 private:
  /**
   * @brief Utility enum for indexing connector segments.
   */
  enum class Idx { FIRST, SECOND, THIRD };

  /**
   * @brief Check whether switching times are strictly non-decreasing.
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the switching times are non-decreasing; otherwise false.
   */
  static bool switchingTimesNonDecreasing(const PST_Connector& connector);

  /**
   * @brief Check whether the function segments of the connector intersect at
   * the switching times.
   *
   * @note An approximate floating point comparison is used.
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the segments are connected; otherwise false.
   */
  static bool segmentsConnected(const PST_Connector& connector);

  /**
   * @brief Check whether the function segments of the connector have equal
   * first derivatives at the switching times.
   *
   * @note An approximate floating point comparison is used.
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the segments have equal first derivatives; otherwise false.
   */
  static bool segmentsTangent(const PST_Connector& connector);

  /**
   * @brief Check whether all segments have valid coefficients.
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the segments all have valid coefficients; otherwise false.
   */
  static bool validSegments(const PST_Connector& connector);

  /**
   * @brief Whether the time domain of the connector has non-zero measure.
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the time domain has non-zero measure; otherwise false.
   */
  static bool timeDomainNonZeroMeasure(const PST_Connector& connector);

  /**
   * @brief Whether the first derivative is non-negative along entire connector.
   *
   * @tparam I The segment index.
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the first derivative along the segment is never negative.
   */
  template <Idx I>
  static bool noNegativeFirstDerivatives(const PST_Connector& connector);

  /**
   * @brief Get a reference to the function for a given segment.
   *
   * @tparam Idx The segment index.
   *
   * @param connector The connecting trajectory.
   *
   * @return A const reference to the function for the given segment.
   */
  template <Idx I>
  static const Polynomial& function(const PST_Connector& connector);

  /**
   * @brief Get an interval domain representation of a segment's domain.
   *
   * @tparam Idx The segment index desired.
   *
   * @param connector The connecting trajectory.
   *
   * @return An interval representing the segment's domain.
   */
  template <Idx I>
  static Interval domain(const PST_Connector& connector);

  /**
   * @brief Perform basic checks for validity of the connecting trajectory.
   *
   * In order to be valid, the following are necessary conditions:
   *
   *   1) The switching times must be strictly non-decreasing.
   *   2) The values and tangents of parabolas at indices 0 and 1 must be equal
   *      at the switching time at index 1.
   *   3) The values and tangents of parabolas at indices 1 and 2 must be equal
   *      at the switching time at index 2.
   *   4) All parabola coefficients must be real valued.
   *   5) First derivative must be non-negative along connector.
   *   6) The total time domain must have non-zero measure.
   *
   * @note These are necessary, not sufficient, conditions for the connector to
   * be valid.
   *
   * @param connecting_trajectory
   *
   * @return True if the connector passes necessary checks; otherwise false.
   */
  static bool valid(const PST_Connector& connector);

  /**
   * @brief Switching times, in order, of the trajectory.
   *
   * The switching times are ordered as follows:
   *
   *   0: Trajectory begins, switches to initial parabolic portion
   *   1: Trajectory switches to linear portion
   *   2: Trajectory switches to second parabolic portion
   *   3: Trajectory ends
   */
  std::array<double, 4> switching_times_;

  /**
   * @brief Ordered functional segments of trajectory.
   *
   * The functional segments are ordered as follows:
   *
   *   0: Initial parabolic segment
   *   1: Interstitial linear segment
   *   2: Terminal parabolic segment
   */
  std::array<Polynomial, 3> functions_;
};  // class PST_Connector

template <PST_Connector::Idx I>
bool PST_Connector::noNegativeFirstDerivatives(const PST_Connector& connector) {
  // Capture the time domain.
  const auto domain = PST_Connector::domain<I>(connector);

  // Capture the function.
  const auto& function = PST_Connector::function<I>(connector);

  // Trivially, if the domain is length zero, or the function is constant, there
  // are no negative first derivatives.
  if (Interval::zeroLength(domain) || Polynomial::isConstant(function)) {
    return true;
  }

  // If quadratic, test accordingly.
  if (Polynomial::isQuadratic(function)) {
    // Check for intersection between the negative domain of function and the
    // segment domain.
    if (const auto dx_partition = Polynomial::dxSignDomainPartition(function)) {
      const auto& I_neg = std::get<0>(*dx_partition);
      const auto i = Interval::intersect(I_neg, domain);

      // This intersection may contain only the critical point, which satisfies
      // the test, but means the interval is not empty. Therefore, check for an
      // intersection of non-zero length in addition to being not empty.
      if (!Interval::empty(i) && !Interval::zeroLength(i)) {
        return false;
      }
    }

    // This should never happen.
    assert(false);
  }

  // If linear, test accordingly.
  if (Polynomial::isLinear(function)) {
    if (Polynomial::dx(function, 0.0) < 0.0) {
      return false;
    }
  }

  // Everything checks out.
  return true;
}

/**
 * Specializations for computing segment domain.
 * @{
 */
template <>
Interval PST_Connector::domain<PST_Connector::Idx::FIRST>(
    const PST_Connector& connector);

template <>
Interval PST_Connector::domain<PST_Connector::Idx::SECOND>(
    const PST_Connector& connector);

template <>
Interval PST_Connector::domain<PST_Connector::Idx::THIRD>(
    const PST_Connector& connector);
/** @} */

/**
 * Specializations for testing segment domains.
 * @{
 */
template <>
const Polynomial& PST_Connector::function<PST_Connector::Idx::FIRST>(
    const PST_Connector& connector);

template <>
const Polynomial& PST_Connector::function<PST_Connector::Idx::SECOND>(
    const PST_Connector& connector);

template <>
const Polynomial& PST_Connector::function<PST_Connector::Idx::THIRD>(
    const PST_Connector& connector);
/** @} */
}  // namespace maeve_automation_core
