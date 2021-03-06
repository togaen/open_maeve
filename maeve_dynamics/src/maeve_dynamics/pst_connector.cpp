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
#include "open_maeve/maeve_dynamics/pst_connector.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "open_maeve/maeve_geometry/comparisons.h"
#include "open_maeve/maeve_geometry/powers.h"

namespace open_maeve {
namespace {
const auto Inf = std::numeric_limits<double>::infinity();
const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

//------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const PST_Connector& connector) {
  os << "{\"parabolic_segments\": ["
     << PST_Connector::function<PST_Connector::Idx::FIRST>(connector) << ", "
     << PST_Connector::function<PST_Connector::Idx::SECOND>(connector) << ", "
     << PST_Connector::function<PST_Connector::Idx::THIRD>(connector) << "]}";
  return os;
}

//------------------------------------------------------------------------------

std::tuple<double, double, double, double> PST_Connector::switchingTimes(
    const PST_Connector& connector) {
  return std::make_tuple(PST_Connector::switching_time<Idx::FIRST>(connector),
                         PST_Connector::switching_time<Idx::SECOND>(connector),
                         PST_Connector::switching_time<Idx::THIRD>(connector),
                         PST_Connector::switching_time<Idx::FOURTH>(connector));
}

//------------------------------------------------------------------------------

double PST_Connector::initialSpeed(const PST_Connector& connector) {
  if (PST_Connector::segmentActive<Idx::FIRST>(connector)) {
    return Polynomial::dx(PST_Connector::function<Idx::FIRST>(connector),
                          PST_Connector::switching_time<Idx::FIRST>(connector));
  }
  if (PST_Connector::segmentActive<Idx::SECOND>(connector)) {
    return Polynomial::dx(
        PST_Connector::function<Idx::SECOND>(connector),
        PST_Connector::switching_time<Idx::SECOND>(connector));
  }
  if (PST_Connector::segmentActive<Idx::THIRD>(connector)) {
    return Polynomial::dx(PST_Connector::function<Idx::THIRD>(connector),
                          PST_Connector::switching_time<Idx::THIRD>(connector));
  }

  // No active segment.
  return NaN;
}

//------------------------------------------------------------------------------

double PST_Connector::terminalSpeed(const PST_Connector& connector) {
  if (PST_Connector::segmentActive<Idx::THIRD>(connector)) {
    return Polynomial::dx(
        PST_Connector::function<Idx::THIRD>(connector),
        PST_Connector::switching_time<Idx::FOURTH>(connector));
  }
  if (PST_Connector::segmentActive<Idx::SECOND>(connector)) {
    return Polynomial::dx(PST_Connector::function<Idx::SECOND>(connector),
                          PST_Connector::switching_time<Idx::THIRD>(connector));
  }
  if (PST_Connector::segmentActive<Idx::FIRST>(connector)) {
    return Polynomial::dx(
        PST_Connector::function<Idx::FIRST>(connector),
        PST_Connector::switching_time<Idx::SECOND>(connector));
  }

  // No active segment.
  return NaN;
}

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

boost::optional<PST_Connector> PST_Connector::computePLP(
    const Eigen::Vector2d& p1, const double p1_dt, const double p1_ddt,
    const Eigen::Vector2d& p2, const double p2_dt, const double p2_ddt,
    const double epsilon) {
  // Compute P segments.
  const auto P1 = Polynomial::from_point_with_derivatives(p1, p1_dt, p1_ddt);
  const auto P2 = Polynomial::from_point_with_derivatives(p2, p2_dt, p2_ddt);

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
  if (const auto roots = Polynomial::roots(A, B, C, epsilon)) {
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
  const auto L1 = Polynomial::from_point_with_derivatives(p2_1, dt_1, 0.0);
  const auto L2 = Polynomial::from_point_with_derivatives(p2_2, dt_2, 0.0);

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

//------------------------------------------------------------------------------

boost::optional<PST_Connector> PST_Connector::computePL_0P(
    const Eigen::Vector2d& p1, const double p1_dt, const double p1_ddt,
    const Eigen::Vector2d& p2, const double p2_ddt, const double epsilon) {
  // Compute initial P.
  const auto P1 = Polynomial::from_point_with_derivatives(p1, p1_dt, p1_ddt);

  // Attempt to compute critical point of P1.
  const auto p_critical = Polynomial::uniqueCriticalPoint(P1);
  if (!p_critical) {
    return boost::none;
  }

  // Attempt to find critical points for candidate terminal parabolas.
  const auto critical_points = Polynomial::findConstrainedCriticalPoints(
      p2, p_critical->y(), p2_ddt, epsilon);
  if (!critical_points) {
    return boost::none;
  }

  // Extract critical points.
  Eigen::Vector2d critical_pt1, critical_pt2;
  std::tie(critical_pt1, critical_pt2) = *critical_points;

  // Build candidate terminal P curves.
  static const auto dx_critical = 0.0;
  const auto P2_candidate1 = Polynomial::from_point_with_derivatives(
      critical_pt1, dx_critical, p2_ddt);
  const auto P2_candidate2 = Polynomial::from_point_with_derivatives(
      critical_pt2, dx_critical, p2_ddt);

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

//------------------------------------------------------------------------------

boost::optional<PST_Connector> PST_Connector::computePP(
    const Eigen::Vector2d& p1, const double p1_dt, const double p1_ddt,
    const Eigen::Vector2d& p2, const double p2_ddt, const double epsilon) {
  // Compute P1 segment coefficients.
  const auto P1 = Polynomial::from_point_with_derivatives(p1, p1_dt, p1_ddt);
  double a1, b1, c1;
  std::tie(a1, b1, c1) = Polynomial::coefficients(P1);

  // Compute tangency point candidates.
  const auto a2 = p2_ddt;
  const auto A = (a1 - a2);
  const auto B = (-2.0 * p2.x() * A);
  const auto C = (-p2.x() * (a2 * p2.x() + b1) - c1 + p2.y());
  Eigen::Vector2d p_t1(NaN, NaN);
  Eigen::Vector2d p_t2(NaN, NaN);
  if (const auto roots = Polynomial::roots(A, B, C, epsilon)) {
    double r1, r2;
    std::tie(r1, r2) = *roots;
    p_t1 = Eigen::Vector2d(r1, P1(r1));
    p_t2 = Eigen::Vector2d(r2, P1(r2));
  } else {
    return boost::none;
  }

  // Compute L candidates (for completeness; not actually necessary).
  constexpr auto L_a = 0.0;
  const auto L1_b = Polynomial::dx(P1, p_t1.x());
  const auto L2_b = Polynomial::dx(P1, p_t2.x());
  const auto L_c = [](const Eigen::Vector2d& p, const double dx) {
    constexpr auto ddx = 0.0;
    const auto poly = Polynomial::from_point_with_derivatives(p, dx, ddx);
    return Polynomial::c(poly);
  };
  const auto L1 = Polynomial(L_a, L1_b, L_c(p_t1, L1_b));
  const auto L2 = Polynomial(L_a, L2_b, L_c(p_t2, L2_b));

  // Compute P2 candidate segment coefficients.
  const auto P2_1 = Polynomial::from_point_with_derivatives(p_t1, L1_b, a2);
  const auto P2_2 = Polynomial::from_point_with_derivatives(p_t2, L2_b, a2);
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

//------------------------------------------------------------------------------

boost::optional<PST_Connector> PST_Connector::computeLP(
    const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const double p2_dt,
    const double p2_ddt) {
  // Compute P portion.
  const auto P = Polynomial::from_point_with_derivatives(p2, p2_dt, p2_ddt);

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

//------------------------------------------------------------------------------

std::tuple<Eigen::Vector2d, Eigen::Vector2d> PST_Connector::boundaryPoints(
    const PST_Connector& connector) {
  const auto t0 = PST_Connector::switching_time<Idx::FIRST>(connector);
  const auto t1 = PST_Connector::switching_time<Idx::FOURTH>(connector);
  const auto& f0 = PST_Connector::function<Idx::FIRST>(connector);
  const auto& f1 = PST_Connector::function<Idx::THIRD>(connector);

  const Eigen::Vector2d p0(t0, f0(t0));
  const Eigen::Vector2d p1(t1, f1(t1));

  return std::make_tuple(p0, p1);
}

//------------------------------------------------------------------------------

bool PST_Connector::timeDomainNonZeroMeasure(const PST_Connector& connector) {
  const auto D =
      Interval<double>(PST_Connector::switching_time<Idx::FIRST>(connector),
                       PST_Connector::switching_time<Idx::FOURTH>(connector));
  return !Interval<double>::zero_length(D);
}

//------------------------------------------------------------------------------

template <>
const Polynomial& PST_Connector::function<PST_Connector::Idx::FIRST>(
    const PST_Connector& connector) {
  return connector.functions_[0];
}

//------------------------------------------------------------------------------

template <>
const Polynomial& PST_Connector::function<PST_Connector::Idx::SECOND>(
    const PST_Connector& connector) {
  return connector.functions_[1];
}

//------------------------------------------------------------------------------

template <>
const Polynomial& PST_Connector::function<PST_Connector::Idx::THIRD>(
    const PST_Connector& connector) {
  return connector.functions_[2];
}

//------------------------------------------------------------------------------

template <>
const double PST_Connector::switching_time<PST_Connector::Idx::FOURTH>(
    const PST_Connector& connector) {
  const auto& P1 = PST_Connector::function<Idx::THIRD>(connector);
  const auto& domain = Polynomial::get_domain(P1);
  return Interval<double>::max(domain);
}

//------------------------------------------------------------------------------

bool PST_Connector::domains_adjacent(const PST_Connector& connector) {
  const auto& d1 =
      Polynomial::get_domain(PST_Connector::function<Idx::FIRST>(connector));
  const auto& d2 =
      Polynomial::get_domain(PST_Connector::function<Idx::SECOND>(connector));
  const auto& d3 =
      Polynomial::get_domain(PST_Connector::function<Idx::THIRD>(connector));

  return (Interval_d::are_adjacent_ordered(d1, d2) &&
          Interval_d::are_adjacent_ordered(d2, d3));
}

//------------------------------------------------------------------------------

bool PST_Connector::segmentsConnected(const PST_Connector& connector) {
  // Record switching times for convenience.
  const auto t1 = PST_Connector::switching_time<Idx::SECOND>(connector);
  const auto t2 = PST_Connector::switching_time<Idx::THIRD>(connector);

  // Compute paths value of segments 0 and 1 at time t1.
  const auto s01 = PST_Connector::function<Idx::FIRST>(connector)(t1);
  const auto s11 = PST_Connector::function<Idx::SECOND>(connector)(t1);

  // Compute path values of segments 1 and 2 at time t2.
  const auto s12 = PST_Connector::function<Idx::SECOND>(connector)(t2);
  const auto s22 = PST_Connector::function<Idx::THIRD>(connector)(t2);

  // Absolute error for doing approximate floating point comparisons.
  // TODO(me): This should be a parameter.
  constexpr auto EPS = 1e-4;

  // The path values at t1 and at t2 should be equal.
  const auto connected =
      (approxRelEq(s01, s11, EPS, EPS) && approxRelEq(s12, s22, EPS, EPS));
#if 0
  if (!connected) {
    const auto rel_diff = std::abs(s01 - s11);
    const auto rel_eps = (std::max(std::abs(s01), std::abs(s11)) * EPS);
    std::cerr << std::setprecision(20) << "rel_diff: " << rel_diff
              << ", rel_eps: " << rel_eps << ", s01: " << s01
              << ", s11: " << s11 << ", s12: " << s12 << ", s22: " << s22
              << std::endl;
  }
#endif
  return connected;
}

//------------------------------------------------------------------------------

bool PST_Connector::segmentsTangent(const PST_Connector& connector) {
  // Record switching times for convenience.
  double t0, t1, t2, t3;
  std::tie(t0, t1, t2, t3) = PST_Connector::switchingTimes(connector);

  // Compute \dot{s} value of segments 0 and 1 at time t1.
  const auto s_dot01 =
      Polynomial::dx(PST_Connector::function<Idx::FIRST>(connector), t1);
  const auto s_dot11 =
      Polynomial::dx(PST_Connector::function<Idx::SECOND>(connector), t1);

  // Compute \dot{s} values of segments 1 and 2 at time t2.
  const auto s_dot12 =
      Polynomial::dx(PST_Connector::function<Idx::SECOND>(connector), t2);
  const auto s_dot22 =
      Polynomial::dx(PST_Connector::function<Idx::THIRD>(connector), t2);

  // Absolute error for doing approximate floating point comparisons.
  // TODO(me): This should be a parameter.
  constexpr auto EPS = 1e-5;

  // The path values at t1 and at t2 should be equal.
  const auto tangent = (approxRelEq(s_dot01, s_dot11, EPS, EPS) &&
                        approxRelEq(s_dot12, s_dot22, EPS, EPS));
#if 0
  if (!tangent) {
    std::cerr << "s_dot01: " << s_dot01 << ", s_dot11: " << s_dot11
              << ", s_dot12: " << s_dot12 << ", s_dot22: " << s_dot22
              << std::endl;
  }
#endif
  return tangent;
}

//------------------------------------------------------------------------------

bool PST_Connector::validSegments(const PST_Connector& connector) {
  return std::all_of(std::begin(connector.functions_),
                     std::end(connector.functions_), Polynomial::valid);
}

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

bool PST_Connector::boundedInteriorTimes(const PST_Connector& connector,
                                         const Interval<double>& bounds) {
  return std::all_of(std::begin(connector.functions_),
                     std::end(connector.functions_), [&](const Polynomial& p) {
                       const auto& domain = Polynomial::get_domain(p);
                       return Interval<double>::is_subset_eq(domain, bounds);
                     });
}

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

std::tuple<bool, std::string> PST_Connector::valid(
    const PST_Connector& connector) {
  // Check validity.
  const auto segments_valid = PST_Connector::validSegments(connector);

  // Check monotonicity.
  const auto adjacent_domains =
      (segments_valid && PST_Connector::domains_adjacent(connector));

  // Check time domain.
  const auto time_domain_valid =
      (adjacent_domains && PST_Connector::timeDomainNonZeroMeasure(connector));

  // Check connectivity.
  const auto segments_connected =
      (adjacent_domains && PST_Connector::segmentsConnected(connector));

  // Check tangency.
  const auto segments_tangent =
      (segments_connected && PST_Connector::segmentsTangent(connector));

  // For debugging.
  std::stringstream ss;
  ss << "segments_valid: " << segments_valid
     << ", adjacent_domains: " << adjacent_domains
     << ", time_domain_valid: " << time_domain_valid
     << ", segments_connected: " << segments_connected
     << ", segments_tangent: " << segments_tangent;

  // Done.
  const auto is_valid =
      (adjacent_domains && segments_connected && segments_tangent &&
       segments_valid && time_domain_valid);
  return std::make_tuple(is_valid, ss.str());
}

//------------------------------------------------------------------------------

PST_Connector::PST_Connector(const std::array<double, 4>& switching_times,
                             const std::array<Polynomial, 3>& functions,
                             const SpeedConstraint speed_constraint)
    : functions_(functions) {
  // Set function domains.
  const auto d1 = Interval<double>(switching_times[0], switching_times[1]);
  const auto d2 = Interval<double>(switching_times[1], switching_times[2]);
  const auto d3 = Interval<double>(switching_times[2], switching_times[3]);
  const auto f1 = Polynomial(functions_[0], d1);
  const auto f2 = Polynomial(functions_[1], d2);
  const auto f3 = Polynomial(functions_[2], d3);
  *this = PST_Connector({f1, f2, f3}, speed_constraint);
}

//------------------------------------------------------------------------------

PST_Connector::PST_Connector(const std::array<Polynomial, 3>& functions,
                             const SpeedConstraint speed_constraint)
    : functions_(functions) {
  // TODO(me): why am I using affine extension to reals here?
  auto speed_bounds = Interval<double>::affinely_extended_reals();
  switch (speed_constraint) {
    case SpeedConstraint::NONE:
      break;
    case SpeedConstraint::STRICTLY_NON_NEGATIVE:
      speed_bounds = Interval<double>::nonnegative_affinely_extended_reals();
      break;
    default:
      throw std::runtime_error(
          "Attempted to build PST connector with unknown speed constraint "
          "type.");
  }

  const auto speeds_valid =
      PST_Connector::boundedInteriorSpeeds(*this, speed_bounds);

  bool obj_valid;
  std::string debug_string;
  std::tie(obj_valid, debug_string) = PST_Connector::valid(*this);

  const auto is_valid = (obj_valid && speeds_valid);

  if (!is_valid) {
    std::stringstream ss;
    ss << "Cannot construct PST Connector for one of the following reasons: "
          "speeds_valid: "
       << speeds_valid << ", or obj_valid: " << obj_valid
       << " (obj_valid debug string: " << debug_string
       << "). PST_Connector constructor was given the following parameter "
          "values: "
       << *this;
    throw std::domain_error(ss.str());
  }
}

//------------------------------------------------------------------------------

boost::optional<PST_Connector> PST_Connector::noExceptionConstructor(
    const std::array<double, 4>& switching_times,
    const std::array<Polynomial, 3>& functions) noexcept {
  try {
    return PST_Connector(switching_times, functions);
  } catch (...) {
    return boost::none;
  }
}

//------------------------------------------------------------------------------

PST_Connector PST_Connector::P(const Polynomial& P_raw,
                               const Interval_d& connector_domain,
                               const SpeedConstraint speed_constraint) {
  const auto P = Polynomial(P_raw, connector_domain);
  const auto t_final = Interval_d::max(connector_domain);
  const auto p_final = P(t_final);
  const Eigen::Vector2d terminal_point(t_final, p_final);

  constexpr auto L_ddx = 0.0;
  const auto L_dx = Polynomial::dx(P, t_final);
  const auto noop_domain = Interval_d(t_final, t_final);
  const auto L_noop = Polynomial::from_point_with_derivatives(
      terminal_point, L_dx, L_ddx, noop_domain);
  return PST_Connector({P, L_noop, L_noop}, speed_constraint);
}

//------------------------------------------------------------------------------

PST_Connector PST_Connector::L(const Polynomial& L_raw,
                               const Interval_d& connector_domain,
                               const SpeedConstraint speed_constraint) {
  const auto L = Polynomial(L_raw, connector_domain);

  const auto t_initial = Interval_d::min(connector_domain);
  const auto p_initial = L(t_initial);
  const Eigen::Vector2d initial_point(t_initial, p_initial);

  const auto t_final = Interval_d::max(connector_domain);
  const auto p_final = L(t_final);
  const Eigen::Vector2d terminal_point(t_final, p_final);

  const auto P_dt = Polynomial::dx(L, t_final);
  const auto P_ddt = 1.0;
  const auto P1_domain = Interval_d(t_initial, t_initial);
  const auto P1 = Polynomial::from_point_with_derivatives(initial_point, P_dt,
                                                          P_ddt, P1_domain);

  const auto P2_domain = Interval_d(t_final, t_final);
  const auto P2 = Polynomial::from_point_with_derivatives(terminal_point, P_dt,
                                                          P_ddt, P2_domain);

  return PST_Connector({P1, L, P2}, speed_constraint);
}

//------------------------------------------------------------------------------

PST_Connector PST_Connector::Pminus_L0(const Polynomial& P_raw,
                                       const Interval_d& connector_domain,
                                       const SpeedConstraint speed_constraint) {
  const auto a_is_nonnegative = (Polynomial::a(P_raw) >= 0.0);
  if (a_is_nonnegative) {
    std::stringstream ss;
    ss << "Cannot construct Pminus_L0. The given function has nonnegative "
          "quadratic term.";
    throw std::runtime_error(ss.str());
  }

  const auto critical_point = Polynomial::uniqueCriticalPoint(P_raw);
  if (!critical_point) {
    std::stringstream ss;
    ss << "Cannot construct Pminus_L0. The given function " << P_raw
       << " has critical point outside the function domain.";
    throw std::runtime_error(ss.str());
  }

  const auto& P_raw_domain = Polynomial::get_domain(P_raw);
  const auto P_domain =
      Interval_d(Interval_d::min(connector_domain), critical_point->x());
  const auto P = Polynomial(P_raw, P_domain);

  const auto L_domain =
      Interval_d(critical_point->x(), Interval_d::max(connector_domain));
  constexpr auto L_dx = 0.0;
  constexpr auto L_ddx = 0.0;
  const auto L = Polynomial::from_point_with_derivatives(*critical_point, L_dx,
                                                         L_ddx, L_domain);

  const auto noop_domain =
      Interval_d(Interval_d::max(L_domain), Interval_d::max(L_domain));
  const auto L_noop = Polynomial(L, noop_domain);
  return PST_Connector({P, L, L_noop}, speed_constraint);
}

//------------------------------------------------------------------------------

bool PST_Connector::is_P(const PST_Connector& connector) {
  const auto P1_active = PST_Connector::segmentActive<Idx::FIRST>(connector);
  const auto L_active = PST_Connector::segmentActive<Idx::SECOND>(connector);
  const auto P2_active = PST_Connector::segmentActive<Idx::THIRD>(connector);
  const auto is_P = (P1_active && !L_active && !P2_active);
  return is_P;
}

//------------------------------------------------------------------------------

bool PST_Connector::is_Pminus(const PST_Connector& connector) {
  const auto is_P = PST_Connector::is_P(connector);
  const auto P1_is_minus =
      (PST_Connector::initialAcceleration(connector) < 0.0);
  return (is_P && P1_is_minus);
}

//------------------------------------------------------------------------------

bool PST_Connector::is_L(const PST_Connector& connector) {
  const auto P1_active = PST_Connector::segmentActive<Idx::FIRST>(connector);
  const auto L_active = PST_Connector::segmentActive<Idx::SECOND>(connector);
  const auto P2_active = PST_Connector::segmentActive<Idx::THIRD>(connector);
  const auto is_L = (!P1_active && L_active && !P2_active);
  return is_L;
}

//------------------------------------------------------------------------------

bool PST_Connector::is_L0(const PST_Connector& connector) {
  const auto is_L = PST_Connector::is_L(connector);

  const auto L = PST_Connector::function<Idx::SECOND>(connector);
  const auto is_zero_slope = Polynomial::is_constant(L);

  return (is_L && is_zero_slope);
}

//------------------------------------------------------------------------------

bool PST_Connector::is_PminusL_0(const PST_Connector& connector) {
  const auto P1_active = PST_Connector::segmentActive<Idx::FIRST>(connector);
  const auto L_active = PST_Connector::segmentActive<Idx::SECOND>(connector);
  const auto P2_active = PST_Connector::segmentActive<Idx::THIRD>(connector);
  const auto is_PL = (P1_active && L_active && !P2_active);

  const auto P1_is_minus =
      (PST_Connector::initialAcceleration(connector) < 0.0);

  return (is_PL && P1_is_minus);
}

//------------------------------------------------------------------------------

}  // namespace open_maeve
