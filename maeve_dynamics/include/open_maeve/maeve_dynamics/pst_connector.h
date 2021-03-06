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
#include <string>
#include <tuple>

#include "boost/optional.hpp"

#include "open_maeve/maeve_dynamics/interval_constraints.h"
#include "open_maeve/maeve_geometry/interval.h"
#include "open_maeve/maeve_geometry/polynomial.h"

namespace open_maeve {
/**
 * @brief This class defines the canonical form of a PST connecting trajectory.
 *
 * @note The speed along the connector is constrained to be strictly
 * non-negative unless explicitly allowed during construction.
 *
 * TODO(me): The connector shouldn't really care whether speeds are negative or
 * not, but dependent libraries already rely on that behavior. Need to refactor
 * so that constraints are passed into the factory methods.
 *
 * TODO(me): This class is over complicated; a lot of streamlining could happen.
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
   * @brief For specifying how speed should be constrained along the connector.
   */
  enum class SpeedConstraint { NONE, STRICTLY_NON_NEGATIVE };

  /**
   * @brief Stream overload for PST Connectors.
   *
   * @note The serialization is JSON compatible.
   *
   * @param os The output stream.
   * @param connector The PST connector object.
   *
   * @return The output stream with the object serialized.
   */
  friend std::ostream& operator<<(std::ostream& os,
                                  const PST_Connector& connector);

  /**
   * @brief Constructor: explicitly initialize the connector.
   *
   * TODO(me): get rid of default parameter value
   *
   * @note This constructor checks for validity of the arguments and throws an
   * exception if they do not meet basic necessary conditions.
   */
  PST_Connector(const std::array<Polynomial, 3>& functions,
                const SpeedConstraint speed_constraint =
                    SpeedConstraint::STRICTLY_NON_NEGATIVE);

  /**
   * @brief Overload for legacy instantiations the specify switching times
   * explicitly.
   *
   * TODO(me): get rid of default parameter value
   *
   * @note The specified switching time override the domains of the polynomials
   * in the 'functions' array.
   */
  PST_Connector(const std::array<double, 4>& switching_times,
                const std::array<Polynomial, 3>& functions,
                const SpeedConstraint speed_constraint =
                    SpeedConstraint::STRICTLY_NON_NEGATIVE);

  /**
   * @brief Disallow default construction.
   */
  PST_Connector() = delete;

  /**
   * @brief Factory method that calls constructor but swallows exception.
   *
   * @param switching_times Trajectory switching times.
   * @param functions Trajectory functional segments.
   *
   * @return A nullable object with either the object or null.
   */
  static boost::optional<PST_Connector> noExceptionConstructor(
      const std::array<double, 4>& switching_times,
      const std::array<Polynomial, 3>& functions) noexcept;

  /**
   * @brief Factory methods for specific types of connectors.
   * @{
   *
   * TODO(me): unit tests
   * TODO(me): these evaluate P at the domain extremum; this can cause numerical
   * issues if the domains are really large (say, DBL_MAX). need to rework these
   * to avoid that.
   */
  static PST_Connector P(const Polynomial& P,
                         const Interval_d& connector_domain,
                         const SpeedConstraint speed_constraint);
  static PST_Connector L(const Polynomial& L,
                         const Interval_d& connector_domain,
                         const SpeedConstraint speed_constraint);
  static PST_Connector Pminus_L0(const Polynomial& P,
                                 const Interval_d& connector_domain,
                                 const SpeedConstraint speed_constraint);
  /** @} */

  /** @brief Check the connector taxonomy. @{ */
  static bool is_P(const PST_Connector& connector);
  static bool is_Pminus(const PST_Connector& connector);
  static bool is_L(const PST_Connector& connector);
  static bool is_L0(const PST_Connector& connector);
  static bool is_PminusL_0(const PST_Connector& connector);
  /** @} */

  /**
   * @brief Get the speed at the beginning of the connector.
   *
   * @param connector The connecting trajectory.
   *
   * @return The initial speed.
   */
  static double initialSpeed(const PST_Connector& connector);

  /**
   * @brief Get the acceleration at the beginning of the connector.
   *
   * @param connector The connecting Trajectory.
   *
   * @return The initial acceleration.
   */
  static double initialAcceleration(const PST_Connector& connector);

  /**
   * @brief Get the speed at the end of the connector.
   *
   * @param connector The connecting trajectory.
   *
   * @return The terminal speed.
   */
  static double terminalSpeed(const PST_Connector& connector);

  /**
   * @brief Get the acceleration at the end of the connector.
   *
   * @param connector The connecting Trajectory.
   *
   * @return The terminal acceleration.
   */
  static double terminalAcceleration(const PST_Connector& connector);

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
   *
   * @return A nullable of object of either the connector or boost::none.
   */
  static boost::optional<PST_Connector> computeLP(const Eigen::Vector2d& p1,
                                                  const Eigen::Vector2d& p2,
                                                  const double p2_dt,
                                                  const double p2_ddt);

  /**
   * @brief Compute a PLP connector between 'p1' and 'p1'.
   *
   * @pre The second derivatives must be non-zero.
   *
   * @param p1 The initial point in PT space.
   * @param p1_dt Connector first derivative at 'p1'.
   * @param p1_ddt Connector second derivative at 'p1'.
   * @param p2 The terminal point in PT space.
   * @param p2_dt Connector first derivative at 'p2'.
   * @param p2_ddt Connector second derivative at 'p2'.
   *
   * @return A nullable object of either the connector or boost::none.
   */
  static boost::optional<PST_Connector> computePLP(
      const Eigen::Vector2d& p1, const double p1_dt, const double p1_ddt,
      const Eigen::Vector2d& p2, const double p2_dt, const double p2_ddt,
      const double epsilon);

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
   *
   * @return A nullable object of either the connector or boost::none.
   */
  static boost::optional<PST_Connector> computePL_0P(
      const Eigen::Vector2d& p1, const double p1_dt, const double p1_ddt,
      const Eigen::Vector2d& p2, const double p2_ddt, const double epsilon);

  /**
   * @brief Compute a PP connector between 'p1' and 'p2'.
   *
   * The PP connector is a special case of the PLP connector who linear portion
   * is inactive, and whose parabolic segments touch at a point of shared
   * tangency. This curve differs from the PL_0P connector in that the points of
   * tangency are not necessarily critical points of the parabolic segments.
   *
   * @param p1 The initial point in PT space.
   * @param p1_dt Connector first derivative at 'p1'.
   * @param p1_ddt Connector second deriviative at 'p1'.
   * @param p2 The terminal point in PT space.
   * @param p2_ddt Connector second derivative at 'p2'.
   *
   * @return A nullable object of either the connector or boost::none.
   */
  static boost::optional<PST_Connector> computePP(
      const Eigen::Vector2d& p1, const double p1_dt, const double p1_ddt,
      const Eigen::Vector2d& p2, const double p2_ddt, const double epsilon);

  /**
   * @brief Check the connector against a set of dynamic constraints.
   *
   * @param connector The connector.
   * @param constraints The set of dynamic constraints.
   *
   * @return True if the connectory is dynamically feasible (satisfies
   * constraints); otherwise false.
   */
  static bool dynamicallyFeasible(
      const PST_Connector& connector,
      const IntervalConstraints<2, double>& constraints);

  /**
   * @brief Test whether times along the connector are bounded.
   *
   * @param connector The connector.
   * @param bounds The position bounds.
   *
   * @return True if all times reached along connector are contained in
   * 'bounds'; otherwise false.
   */
  static bool boundedInteriorTimes(const PST_Connector& connector,
                                   const Interval<double>& bounds);

  /**
   * @brief Test whether positions along the connector are bounded.
   *
   * @param connector The connector.
   * @param bounds The position bounds.
   *
   * @return True if all positions reached along connector are contained in
   * 'bounds'; otherwise false.
   */
  static bool boundedInteriorPositions(const PST_Connector& connector,
                                       const Interval<double>& bounds);

  /**
   * @brief Test whether speeds along the connector are bounded.
   *
   * @param connector The connector.
   * @param bounds The speed bounds.
   *
   * @return True if all speeds reached along connector are contained in
   * 'bounds'; otherwise false.
   */
  static bool boundedInteriorSpeeds(const PST_Connector& connector,
                                    const Interval<double>& bounds);

  /**
   * @brief Test whether accelerations along the connector are bounded.
   *
   * @param connector The connector.
   * @param bounds The accelerations bounds.
   *
   * @return True if all accelerations reached along connector are contained in
   * 'bounds'; otherwise false.
   */
  static bool boundedInteriorAccelerations(const PST_Connector& connector,
                                           const Interval<double>& bounds);

  /**
   * @brief Utility enum for indexing components.
   */
  enum class Idx { FIRST, SECOND, THIRD, FOURTH };

  /**
   * @brief Get an interval representation of a segment's range.
   *
   * @tparam I The segment index desired.
   *
   * @param connector The connecting trajectory.
   *
   * @return An interval representing the segment's range.
   */
  template <Idx I>
  static Interval<double> range(const PST_Connector& connector);

  /**
   * @brief Get a reference to the function for a given segment.
   *
   * @tparam I The segment index.
   *
   * @param connector The connecting trajectory.
   *
   * @return A const reference to the function for the given segment.
   */
  template <Idx I>
  static const Polynomial& function(const PST_Connector& connector);

  /**
   * @brief Convenience method for retrieving all switching times as doubles.
   *
   * @param connector The connecting trajectory.
   *
   * @return A tuple of ordered switching times.
   */
  static std::tuple<double, double, double, double> switchingTimes(
      const PST_Connector& connector);

 private:
  /**
   * @brief Get the desired switching time.
   *
   * @tparam I The switching time index.
   *
   * @param connector The connecting trajectory.
   *
   * @return The switching time.
   */
  template <Idx I>
  static const double switching_time(const PST_Connector& connector);

  /**
   * @brief Test whether the time domains are adjacent in order, i.e., overlap
   * exactly at their boundaries.
   *
   * TODO(me): return an error code instead of bool
   *
   * @return True iff time domains are adjacent.
   */
  static bool domains_adjacent(const PST_Connector& connector);

  /**
   * @brief Check whether the function segments of the connector intersect at
   * the switching times.
   *
   * @note An approximate floating point comparison is used.
   *
   * TODO(me): return an error code instead of bool
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
   * TODO(me): return an error code instead of bool
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the segments have equal first derivatives; otherwise false.
   */
  static bool segmentsTangent(const PST_Connector& connector);

  /**
   * @brief Check whether all segments have valid coefficients.
   *
   * TODO(me): return an error code instead of bool
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
   * @brief Whether the zeroth derivative is within given bounds along entire
   * connector.
   *
   * @tparam I The segment index.
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the zeroth derivative along the segment is always withing
   * bounds; otherwise false.
   */
  template <Idx I>
  static bool boundedZerothDerivatives(const PST_Connector& connector,
                                       const Interval<double>& bounds);

  /**
   * @brief Whether the first derivative is within given bounds along entire
   * connector.
   *
   * @tparam I The segment index.
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the first derivative along the segment is always withing
   * bounds; otherwise false.
   */
  template <Idx I>
  static bool boundedFirstDerivatives(const PST_Connector& connector,
                                      const Interval<double>& bounds);

  /**
   * @brief Whether the second derivative is within given bounds along entire
   * connector.
   *
   * @tparam I The segment index.
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the second derivative along the segment is always withing
   * bounds; otherwise false.
   */
  template <Idx I>
  static bool boundedSecondDerivatives(const PST_Connector& connector,
                                       const Interval<double>& bounds);

  /**
   * @brief Whether a given segment is active.
   *
   * A segment is active when its domain has length > 0.
   *
   * @tparam I The segment index desired.
   *
   * @param connector The connecting trajectory.
   *
   * @return True if the segment is active; otherwise false.
   */
  template <Idx I>
  static bool segmentActive(const PST_Connector& connector);

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
   *   5) The total time domain must have non-zero measure.
   *
   * @note These are necessary, not sufficient, conditions for the connector to
   * be valid.
   *
   * @param connecting_trajectory
   *
   * @return True if the connector passes necessary checks; otherwise false.
   */
  static std::tuple<bool, std::string> valid(const PST_Connector& connector);

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

//------------------------------------------------------------------------------

template <PST_Connector::Idx I>
bool PST_Connector::boundedZerothDerivatives(const PST_Connector& connector,
                                             const Interval<double>& bounds) {
  // Construct position range.
  const auto range = PST_Connector::range<I>(connector);

  // Check range.
  return Interval<double>::is_subset_eq(range, bounds);
}

//------------------------------------------------------------------------------

template <PST_Connector::Idx I>
bool PST_Connector::boundedFirstDerivatives(const PST_Connector& connector,
                                            const Interval<double>& bounds) {
  // Capture the function.
  const auto& function = PST_Connector::function<I>(connector);

  // Capture the time domain.
  const auto& domain =
      Polynomial::get_domain(PST_Connector::function<I>(connector));

  // Compute dx at domain bounds.
  const auto s_dot1 = Polynomial::dx(function, Interval<double>::min(domain));
  const auto s_dot2 = Polynomial::dx(function, Interval<double>::max(domain));

  // Construct speed range.
  const auto range =
      Interval<double>(std::min(s_dot1, s_dot2), std::max(s_dot1, s_dot2));

  // Check range.
  return Interval<double>::is_subset_eq(range, bounds);
}

//------------------------------------------------------------------------------

template <PST_Connector::Idx I>
bool PST_Connector::boundedSecondDerivatives(const PST_Connector& connector,
                                             const Interval<double>& bounds) {
  // Capture the function.
  const auto& function = PST_Connector::function<I>(connector);

  // Compute dx at domain bounds.
  const auto s_ddot1 = Polynomial::ddx(function);
  const auto s_ddot2 = Polynomial::ddx(function);

  // Construct acceleration range.
  const auto range =
      Interval<double>(std::min(s_ddot1, s_ddot2), std::max(s_ddot1, s_ddot2));

  // Check range.
  return Interval<double>::is_subset_eq(range, bounds);
}

//------------------------------------------------------------------------------

template <PST_Connector::Idx I>
bool PST_Connector::segmentActive(const PST_Connector& connector) {
  const auto& D = Polynomial::get_domain(PST_Connector::function<I>(connector));
  return !Interval<double>::zero_length(D);
}

//------------------------------------------------------------------------------

template <PST_Connector::Idx I>
Interval<double> PST_Connector::range(const PST_Connector& connector) {
  const auto& f = PST_Connector::function<I>(connector);
  const auto& d = Polynomial::get_domain(PST_Connector::function<I>(connector));
  const auto r1 = f(Interval<double>::min(d));
  const auto r2 = f(Interval<double>::max(d));
  return Interval<double>(std::min(r1, r2), std::max(r1, r2));
}

/**
 * Specializations for getting segment functions.
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

/**
 * @brief Accessors for switching times.
 *
 * @note The fourth time needs a specialization due to different behavior.
 *
 * The switching times are ordered as follows:
 *
 *   1: Trajectory begins, switches to initial parabolic portion
 *   2: Trajectory switches to linear portion
 *   3: Trajectory switches to second parabolic portion
 *   4: Trajectory ends
 *
 * @{
 */
template <>
const double PST_Connector::switching_time<PST_Connector::Idx::FOURTH>(
    const PST_Connector& connector);

template <PST_Connector::Idx I>
const double PST_Connector::switching_time(const PST_Connector& connector) {
  const auto& P1 = PST_Connector::function<I>(connector);
  const auto& domain = Polynomial::get_domain(P1);
  return Interval<double>::min(domain);
}
/** @} */
}  // namespace open_maeve
