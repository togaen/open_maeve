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

namespace maeve_automation_core {
std::ostream& operator<<(std::ostream& os, const PST_Connector& connector) {
  const auto& t = connector.switching_times_;
  const auto& p = connector.functions_;

  os << "{switching times: [" << t[0] << ", " << t[1] << ", " << t[2] << ", "
     << t[3] << "], ";
  os << "parabola coefficients: [" << p[0] << ", " << p[1] << ", " << p[2]
     << "]}";
  return os;
}

double PST_Connector::initialSpeed(const PST_Connector& connector) {
  return Polynomial::dx(connector.functions_[0], connector.switching_times_[0]);
}

double PST_Connector::terminalSpeed(const PST_Connector& connector) {
  return Polynomial::dx(connector.functions_[2], connector.switching_times_[3]);
}

boost::optional<PST_Connector> PST_Connector::computePLP(
    const Eigen::Vector2d& p1, const double p1_dt, const Eigen::Vector2d& p2,
    const double p2_dt, const double p2_ddt, const Interval& I_dt) {
  return boost::none;
}

boost::optional<PST_Connector> PST_Connector::computePL_0P(
    const Eigen::Vector2d& p1, const double p1_dt, const double p1_ddt,
    const Eigen::Vector2d& p2, const double p2_ddt, const Interval& I_dt) {
  // Compute critical point of P1.

  // Construct L_0 through P1 critical point.

  // Compute P2 through p2 with critical point on L_0.

  // Done.
  return boost::none;
}

boost::optional<PST_Connector> PST_Connector::computeLP(
    const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const double p2_dt,
    const double p2_ddt, const Interval& I_dt) {
  // Compute P portion.
  const auto P = Polynomial::fromPointWithDerivatives(p2, p2_dt, p2_ddt);

  // Compute tangent points of L portion.
  const auto rays = Polynomial::tangentRaysThroughPoint(P, p1);
  if (!rays) {
    return boost::none;
  }

  // Construct linear segments to test.
  const auto L1 = Polynomial(p1, std::get<0>(*rays));
  const auto L2 = Polynomial(p1, std::get<1>(*rays));

  // Check L portion.
  const auto s1_dot = Polynomial::dx(L1, p1.x());
  const auto s2_dot = Polynomial::dx(L2, p1.x());
  const auto L1_valid = Interval::contains(I_dt, s1_dot);
  const auto L2_valid = Interval::contains(I_dt, s2_dot);

  // No connection of this type.
  if (!L1_valid && !L2_valid) {
    return boost::none;
  }

  // This should never happen.
  assert(!(L1_valid && L2_valid));

  // For simplicity.
  const auto& L = (L1_valid ? L1 : L2);
  const auto& r = (L1_valid ? std::get<0>(*rays) : std::get<1>(*rays));

  // Build connector and return.
  return PST_Connector({p1.x(), p1.x(), r.x(), p2.x()}, {L, L, P});
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
  const auto t1 = connector.switching_times_[1];
  const auto t2 = connector.switching_times_[2];

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

bool PST_Connector::realCoefficients(const PST_Connector& connector) {
  return std::all_of(std::begin(connector.functions_),
                     std::end(connector.functions_), [](const Polynomial& p) {
                       return std::isfinite(Polynomial::a(p)) &&
                              std::isfinite(Polynomial::b(p)) &&
                              std::isfinite(Polynomial::c(p));
                     });
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
  const auto real_coefficients = PST_Connector::realCoefficients(connector);

  // Done.
  return non_decreasing && segments_connected && segments_tangent &&
         real_coefficients;
}

PST_Connector::PST_Connector(std::array<double, 4>&& switching_times,
                             std::array<Polynomial, 3>&& functions)
    : switching_times_(std::move(switching_times)),
      functions_(std::move(functions)) {
  const auto is_valid = PST_Connector::valid(*this);
  if (!is_valid) {
    std::stringstream ss;
    ss << *this;
    throw std::domain_error("Invalid parameter values for PST connector: " +
                            ss.str());
  }
}
}  // namespace maeve_automation_core
