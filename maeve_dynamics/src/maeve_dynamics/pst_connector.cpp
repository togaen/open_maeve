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
#include "maeve_automation_core/maeve_dynamics/pst_connector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "maeve_automation_core/maeve_geometry/comparisons.h"
#include "maeve_automation_core/maeve_geometry/powers.h"

namespace maeve_automation_core {
namespace {
const auto Inf = std::numeric_limits<double>::infinity();
const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

std::ostream& operator<<(std::ostream& os, const PST_Connector& connector) {
  const auto& t = connector.switching_times_;
  const auto& p = connector.functions_;

  os << "{\"switching_times\": [" << t[0] << ", " << t[1] << ", " << t[2]
     << ", " << t[3] << "], ";
  os << "\"parabola_coefficients\": [" << p[0] << ", " << p[1] << ", " << p[2]
     << "]}";
  return os;
}

std::tuple<double, double, double, double> PST_Connector::switchingTimes(
    const PST_Connector& connector) {
  return std::make_tuple(
      connector.switching_times_[0], connector.switching_times_[1],
      connector.switching_times_[2], connector.switching_times_[3]);
}

double PST_Connector::initialSpeed(const PST_Connector& connector) {
  if (PST_Connector::segmentActive<Idx::FIRST>(connector)) {
    return Polynomial::dx(PST_Connector::function<Idx::FIRST>(connector),
                          connector.switching_times_[0]);
  }
  if (PST_Connector::segmentActive<Idx::SECOND>(connector)) {
    return Polynomial::dx(PST_Connector::function<Idx::SECOND>(connector),
                          connector.switching_times_[1]);
  }
  if (PST_Connector::segmentActive<Idx::THIRD>(connector)) {
    return Polynomial::dx(PST_Connector::function<Idx::THIRD>(connector),
                          connector.switching_times_[2]);
  }

  // No active segment.
  return NaN;
}

double PST_Connector::terminalSpeed(const PST_Connector& connector) {
  if (PST_Connector::segmentActive<Idx::THIRD>(connector)) {
    return Polynomial::dx(PST_Connector::function<Idx::THIRD>(connector),
                          connector.switching_times_[3]);
  }
  if (PST_Connector::segmentActive<Idx::SECOND>(connector)) {
    return Polynomial::dx(PST_Connector::function<Idx::SECOND>(connector),
                          connector.switching_times_[2]);
  }
  if (PST_Connector::segmentActive<Idx::FIRST>(connector)) {
    return Polynomial::dx(PST_Connector::function<Idx::FIRST>(connector),
                          connector.switching_times_[1]);
  }

  // No active segment.
  return NaN;
}

double PST_Connector::initialAcceleration(const PST_Connector& connector) {
  if (PST_Connector::segmentActive<Idx::FIRST>(connector)) {
    return Polynomial::ddx(PST_Connector::function<Idx::FIRST>(connector));
  }
  if (PST_Connector::segmentActive<Idx::SECOND>(connector)) {
    return Polynomial::ddx(PST_Connector::function<Idx::SECOND>(connector));
  }
  if (PST_Connector::segmentActive<Idx::THIRD>(connector)) {
    return Polynomial::ddx(PST_Connector::function<Idx::THIRD>(connector));
  }

  // No active segment.
  return NaN;
}

double PST_Connector::terminalAcceleration(const PST_Connector& connector) {
  if (PST_Connector::segmentActive<Idx::THIRD>(connector)) {
    return Polynomial::ddx(PST_Connector::function<Idx::THIRD>(connector));
  }
  if (PST_Connector::segmentActive<Idx::SECOND>(connector)) {
    return Polynomial::ddx(PST_Connector::function<Idx::SECOND>(connector));
  }
  if (PST_Connector::segmentActive<Idx::FIRST>(connector)) {
    return Polynomial::ddx(PST_Connector::function<Idx::FIRST>(connector));
  }

  // No active segment.
  return NaN;
}

boost::optional<PST_Connector> PST_Connector::computePLP(
    const Eigen::Vector2d& p1, const double p1_dt, const double p1_ddt,
    const Eigen::Vector2d& p2, const double p2_dt, const double p2_ddt) {
  // Compute P segments.
  const auto P1 = Polynomial::fromPointWithDerivatives(p1, p1_dt, p1_ddt);
  const auto P2 = Polynomial::fromPointWithDerivatives(p2, p2_dt, p2_ddt);

  // Get segment coefficients.
  double a0, b0, c0, a3, b3, c3;
  std::tie(a0, b0, c0) = Polynomial::coefficients(P1);
  std::tie(a3, b3, c3) = Polynomial::coefficients(P2);

  // Compute roots for tangency points.
  const auto A = ((square(a3) / a0) - a3);
  const auto B = (a3 * ((b3 - b0) / a0));
  const auto C = (square(b3 - b0) / (4.0 * a0) - c0 + c3);
  Eigen::Vector2d p2_1(NaN, NaN);
  Eigen::Vector2d p2_2(NaN, NaN);
  if (const auto roots = Polynomial::roots(A, B, C)) {
    double r1, r2;
    std::tie(r1, r2) = *roots;
    p2_1 = Eigen::Vector2d(r1, P2(r1));
    p2_2 = Eigen::Vector2d(r2, P2(r2));
  } else {
    return boost::none;
  }

  // Candidate linear portions.
  const auto dt_1 = Polynomial::dx(P2, p2_1.x());
  const auto dt_2 = Polynomial::dx(P2, p2_2.x());
  const auto L1 = Polynomial::fromPointWithDerivatives(p2_1, dt_1, 0.0);
  const auto L2 = Polynomial::fromPointWithDerivatives(p2_2, dt_2, 0.0);

  // Other tangency points.
  const auto p1_1 = Polynomial::quadraticPointAtDerivative(P1, dt_1);
  const auto p1_2 = Polynomial::quadraticPointAtDerivative(P1, dt_2);

  // Candidate connectors.
  const auto C1 = PST_Connector::noExceptionConstructor(
      {p1.x(), p1_1.x(), p2_1.x(), p2.x()}, {P1, L1, P2});
  const auto C2 = PST_Connector::noExceptionConstructor(
      {p1.x(), p1_2.x(), p2_2.x(), p2.x()}, {P1, L2, P2});

  // No valid connector.
  if (!C1 && !C2) {
    return boost::none;
  }

  // This should only happen if the roots are unique.
  if (C1 && C2) {
    if (p2_1.x() == p2_2.x()) {
      return *C1;
    }
    throw std::range_error("Too many valid PLP connectors.");
  }

  // Done.
  return (C1 ? *C1 : *C2);
}

template <>
Interval<double> PST_Connector::domain<PST_Connector::Idx::FIRST>(
    const PST_Connector& connector) {
  return Interval<double>(connector.switching_times_[0],
                          connector.switching_times_[1]);
}

template <>
Interval<double> PST_Connector::domain<PST_Connector::Idx::SECOND>(
    const PST_Connector& connector) {
  return Interval<double>(connector.switching_times_[1],
                          connector.switching_times_[2]);
}

template <>
Interval<double> PST_Connector::domain<PST_Connector::Idx::THIRD>(
    const PST_Connector& connector) {
  return Interval<double>(connector.switching_times_[2],
                          connector.switching_times_[3]);
}

boost::optional<PST_Connector> PST_Connector::computePL_0P(
    const Eigen::Vector2d& p1, const double p1_dt, const double p1_ddt,
    const Eigen::Vector2d& p2, const double p2_ddt) {
  // Compute initial P.
  const auto P1 = Polynomial::fromPointWithDerivatives(p1, p1_dt, p1_ddt);

  // Attempt to compute critical point of P1.
  const auto p_critical = Polynomial::uniqueCriticalPoint(P1);
  if (!p_critical) {
    return boost::none;
  }

  // Attempt to find critical points for candidate terminal parabolas.
  const auto critical_points =
      Polynomial::findConstrainedCriticalPoints(p2, p_critical->y(), p2_ddt);
  if (!critical_points) {
    return boost::none;
  }

  // Extract critical points.
  Eigen::Vector2d critical_pt1, critical_pt2;
  std::tie(critical_pt1, critical_pt2) = *critical_points;

  // Build candidate terminal P curves.
  static const auto dx_critical = 0.0;
  const auto P2_candidate1 =
      Polynomial::fromPointWithDerivatives(critical_pt1, dx_critical, p2_ddt);
  const auto P2_candidate2 =
      Polynomial::fromPointWithDerivatives(critical_pt2, dx_critical, p2_ddt);

  // Build L curve by artificially constructing a line through 'p_critical'.
  const Eigen::Vector2d p_critical2 = (*p_critical + Eigen::Vector2d(1.0, 0.0));
  const auto L = Polynomial(*p_critical, p_critical2);

  // Attempt to build connector and return.
  const auto t0 = p1.x();
  const auto t1 = p_critical->x();
  const auto t3 = p2.x();

  const auto C1 = PST_Connector::noExceptionConstructor(
      {t0, t1, critical_pt1.x(), t3}, {P1, L, P2_candidate1});

  const auto C2 = PST_Connector::noExceptionConstructor(
      {t0, t1, critical_pt2.x(), t3}, {P1, L, P2_candidate2});

  if (C1) {
    return *C1;
  }
  if (C2) {
    return *C2;
  }

  return boost::none;
}

boost::optional<PST_Connector> PST_Connector::computePP(
    const Eigen::Vector2d& p1, const double p1_dt, const double p1_ddt,
    const Eigen::Vector2d& p2, const double p2_ddt) {
  // Compute P1 segment coefficients.
  const auto P1 = Polynomial::fromPointWithDerivatives(p1, p1_dt, p1_ddt);
  double a1, b1, c1;
  std::tie(a1, b1, c1) = Polynomial::coefficients(P1);

  // Compute tangency point candidates.
  const auto a2 = p2_ddt;
  const auto A = (a1 - a2);
  const auto B = (-2.0 * p2.x() * A);
  const auto C = (-p2.x() * (a2 * p2.x() + b1) - c1 + p2.y());
  Eigen::Vector2d p_t1(NaN, NaN);
  Eigen::Vector2d p_t2(NaN, NaN);
  if (const auto roots = Polynomial::roots(A, B, C)) {
    double r1, r2;
    std::tie(r1, r2) = *roots;
    p_t1 = Eigen::Vector2d(r1, P1(r1));
    p_t2 = Eigen::Vector2d(r2, P1(r2));
  } else {
    return boost::none;
  }

  // Compute L candidates (for completeness; not actually necessary).
  static const auto L_c = [](const Eigen::Vector2d& p, const double m) {
    return (p.y() - m * p.x());
  };
  const auto L1_b = Polynomial::dx(P1, p_t1.x());
  const auto L2_b = Polynomial::dx(P1, p_t2.x());
  const auto L1 = Polynomial(0.0, L1_b, L_c(p_t1, L1_b));
  const auto L2 = Polynomial(0.0, L2_b, L_c(p_t2, L2_b));

  // Compute P2 candidate segment coefficients.
  const auto P2_1 = Polynomial::fromPointWithDerivatives(p_t1, L1_b, a2);
  const auto P2_2 = Polynomial::fromPointWithDerivatives(p_t2, L2_b, a2);
  // Find valid connectors, if any.

  const auto C1 = PST_Connector::noExceptionConstructor(
      {p1.x(), p_t1.x(), p_t1.x(), p2.x()}, {P1, L1, P2_1});
  const auto C2 = PST_Connector::noExceptionConstructor(
      {p1.x(), p_t2.x(), p_t2.x(), p2.x()}, {P1, L2, P2_2});

  // This should only happen if the roots are equal.
  if (C1 && C2) {
    if (p_t1.x() == p_t2.x()) {
      return *C1;
    }
    throw std::range_error("Too many valid PP connectors.");
  }

  // Nothing found.
  if (!C1 && !C2) {
    return boost::none;
  }

  // Done.
  return (C1 ? *C1 : *C2);
}

boost::optional<PST_Connector> PST_Connector::computeLP(
    const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const double p2_dt,
    const double p2_ddt) {
  // Compute P portion.
  const auto P = Polynomial::fromPointWithDerivatives(p2, p2_dt, p2_ddt);

  // Compute tangent points of L portion.
  const auto rays = Polynomial::tangentRaysThroughPoint(P, p1);
  if (!rays) {
    return boost::none;
  }

  // Capture ray points.
  Eigen::Vector2d r1, r2;
  std::tie(r1, r2) = *rays;

  // Construct linear segments to test.
  const auto L1 = Polynomial(p1, r1);
  const auto L2 = Polynomial(p1, r2);

  // Attempt to build connector and return.
  const auto C1 = PST_Connector::noExceptionConstructor(
      {p1.x(), p1.x(), r1.x(), p2.x()}, {L1, L1, P});
  const auto C2 = PST_Connector::noExceptionConstructor(
      {p1.x(), p1.x(), r2.x(), p2.x()}, {L2, L2, P});

  if (C1) {
    return *C1;
  }
  if (C2) {
    return *C2;
  }

  return boost::none;
}

std::tuple<Eigen::Vector2d, Eigen::Vector2d> PST_Connector::boundaryPoints(
    const PST_Connector& connector) {
  const auto t0 = connector.switching_times_[0];
  const auto t1 = connector.switching_times_[3];
  const auto& f0 = connector.functions_[0];
  const auto& f1 = connector.functions_[2];

  const Eigen::Vector2d p0(t0, f0(t0));
  const Eigen::Vector2d p1(t1, f1(t1));

  return std::make_tuple(p0, p1);
}

bool PST_Connector::timeDomainNonZeroMeasure(const PST_Connector& connector) {
  const auto D = Interval<double>(connector.switching_times_[0],
                                  connector.switching_times_[3]);
  return !Interval<double>::zero_length(D);
}

template <>
const Polynomial& PST_Connector::function<PST_Connector::Idx::FIRST>(
    const PST_Connector& connector) {
  return connector.functions_[0];
}

template <>
const Polynomial& PST_Connector::function<PST_Connector::Idx::SECOND>(
    const PST_Connector& connector) {
  return connector.functions_[1];
}

template <>
const Polynomial& PST_Connector::function<PST_Connector::Idx::THIRD>(
    const PST_Connector& connector) {
  return connector.functions_[2];
}

bool PST_Connector::switchingTimesNonDecreasing(
    const PST_Connector& connector) {
  // Check monotonicity.
  auto non_decreasing = true;
  for (auto i = 1; i < connector.switching_times_.size(); ++i) {
    non_decreasing = (non_decreasing && (connector.switching_times_[i] >=
                                         connector.switching_times_[i - 1]));
  }
  return non_decreasing;
}

bool PST_Connector::segmentsConnected(const PST_Connector& connector) {
  // Absolute error for doing approximate floating point comparisons.
  static const auto epsilon = 0.00001;

  // Record switching times for convenience.
  const auto t1 = connector.switching_times_[1];
  const auto t2 = connector.switching_times_[2];

  // Compute paths value of segments 0 and 1 at time t1.
  const auto s01 = connector.functions_[0](t1);
  const auto s11 = connector.functions_[1](t1);

  // Compute path values of segments 1 and 2 at time t2.
  const auto s12 = connector.functions_[1](t2);
  const auto s22 = connector.functions_[2](t2);

  // The path values at t1 and at t2 should be equal.
  return approxEq(s01, s11, epsilon) && approxEq(s12, s22, epsilon);
}

bool PST_Connector::segmentsTangent(const PST_Connector& connector) {
  // Absolute error for doing approximate floating point comparisons.
  static const auto epsilon = 0.00001;

  // Record switching times for convenience.
  double t0, t1, t2, t3;
  std::tie(t0, t1, t2, t3) = PST_Connector::switchingTimes(connector);

  // Compute \dot{s} value of segments 0 and 1 at time t1.
  const auto s_dot01 = Polynomial::dx(connector.functions_[0], t1);
  const auto s_dot11 = Polynomial::dx(connector.functions_[1], t1);

  // Compute \dot{s} values of segments 1 and 2 at time t2.
  const auto s_dot12 = Polynomial::dx(connector.functions_[1], t2);
  const auto s_dot22 = Polynomial::dx(connector.functions_[2], t2);

  // The path values at t1 and at t2 should be equal.
  return approxEq(s_dot01, s_dot11, epsilon) &&
         approxEq(s_dot12, s_dot22, epsilon);
}

bool PST_Connector::validSegments(const PST_Connector& connector) {
  return std::all_of(std::begin(connector.functions_),
                     std::end(connector.functions_), [](const Polynomial& p) {
                       return (Polynomial::valid(p) && Polynomial::valid(p) &&
                               Polynomial::valid(p));
                     });
}

bool PST_Connector::boundedInteriorAccelerations(
    const PST_Connector& connector, const Interval<double>& bounds) {
  const auto seg1_valid_ddx =
      PST_Connector::boundedSecondDerivatives<Idx::FIRST>(connector, bounds);
  const auto seg2_valid_ddx =
      PST_Connector::boundedSecondDerivatives<Idx::SECOND>(connector, bounds);
  const auto seg3_valid_ddx =
      PST_Connector::boundedSecondDerivatives<Idx::THIRD>(connector, bounds);

  return (seg1_valid_ddx && seg2_valid_ddx && seg3_valid_ddx);
}

bool PST_Connector::boundedInteriorSpeeds(const PST_Connector& connector,
                                          const Interval<double>& bounds) {
  const auto seg1_valid_dx =
      PST_Connector::boundedFirstDerivatives<Idx::FIRST>(connector, bounds);
  const auto seg2_valid_dx =
      PST_Connector::boundedFirstDerivatives<Idx::SECOND>(connector, bounds);
  const auto seg3_valid_dx =
      PST_Connector::boundedFirstDerivatives<Idx::THIRD>(connector, bounds);

  return (seg1_valid_dx && seg2_valid_dx && seg3_valid_dx);
}

bool PST_Connector::boundedInteriorPositions(const PST_Connector& connector,
                                             const Interval<double>& bounds) {
  const auto seg1_valid =
      PST_Connector::boundedZerothDerivatives<Idx::FIRST>(connector, bounds);
  const auto seg2_valid =
      PST_Connector::boundedZerothDerivatives<Idx::SECOND>(connector, bounds);
  const auto seg3_valid =
      PST_Connector::boundedZerothDerivatives<Idx::THIRD>(connector, bounds);

  return (seg1_valid && seg2_valid && seg3_valid);
}

bool PST_Connector::boundedInteriorTimes(const PST_Connector& connector,
                                         const Interval<double>& bounds) {
  return std::all_of(std::begin(connector.switching_times_),
                     std::end(connector.switching_times_), [&](const double t) {
                       return Interval<double>::contains(bounds, t);
                     });
}

bool PST_Connector::dynamicallyFeasible(
    const PST_Connector& connector,
    const IntervalConstraints<2, double>& constraints) {
  const auto& padding = IntervalConstraints<2, double>::epsilon(constraints);
  const auto& time_bounds =
      IntervalConstraints<2, double>::boundsT(constraints);
  const auto& s_bounds =
      IntervalConstraints<2, double>::boundsS<0>(constraints);
  const auto& s_dot_bounds =
      IntervalConstraints<2, double>::boundsS<1>(constraints);
  const auto& s_ddot_bounds =
      IntervalConstraints<2, double>::boundsS<2>(constraints);

  const auto times_valid =
      PST_Connector::boundedInteriorTimes(connector, time_bounds);
  const auto positions_valid =
      PST_Connector::boundedInteriorPositions(connector, (s_bounds + padding));
  const auto speeds_valid =
      PST_Connector::boundedInteriorSpeeds(connector, (s_dot_bounds + padding));
  const auto accelerations_valid = PST_Connector::boundedInteriorAccelerations(
      connector, (s_ddot_bounds + padding));

  return (times_valid && positions_valid && speeds_valid &&
          accelerations_valid);
}

bool PST_Connector::valid(const PST_Connector& connector) {
  // Check monotonicity.
  const auto non_decreasing =
      PST_Connector::switchingTimesNonDecreasing(connector);

  // Check connectivity.
  const auto segments_connected = PST_Connector::segmentsConnected(connector);

  // Check tangency.
  const auto segments_tangent = PST_Connector::segmentsTangent(connector);

  // Check validity.
  const auto segments_valid = PST_Connector::validSegments(connector);

  // Check first derivatives.
  const auto non_negative =
      Interval<double>::nonnegative_affinely_extended_reals();
  const auto valid_speeds =
      PST_Connector::boundedInteriorSpeeds(connector, non_negative);

  // Check time domain.
  const auto time_domain_valid =
      PST_Connector::timeDomainNonZeroMeasure(connector);
#if 0  // For debugging.
  std::cout << "non_decreasing: " << non_decreasing
            << ", segments_connected: " << segments_connected
            << ", segments_tangent: " << segments_tangent
            << ", segments_valid: " << segments_valid
            << ", valid_speeds: " << valid_speeds
            << ", time_domain_valid: " << time_domain_valid << std::endl;
#endif

  // Done.
  return (non_decreasing && segments_connected && segments_tangent &&
          segments_valid && valid_speeds && time_domain_valid);
}

PST_Connector::PST_Connector(const std::array<double, 4>& switching_times,
                             const std::array<Polynomial, 3>& functions)
    : switching_times_(switching_times), functions_(functions) {
  const auto is_valid = PST_Connector::valid(*this);
  if (!is_valid) {
    std::stringstream ss;
    ss << *this;
    throw std::domain_error("Invalid parameter values for PST connector: " +
                            ss.str());
  }
}

boost::optional<PST_Connector> PST_Connector::noExceptionConstructor(
    const std::array<double, 4>& switching_times,
    const std::array<Polynomial, 3>& functions) noexcept {
  try {
    return PST_Connector(switching_times, functions);
  } catch (...) {
    return boost::none;
  }
}
}  // namespace maeve_automation_core
