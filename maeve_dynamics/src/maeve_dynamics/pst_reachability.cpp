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
#include "maeve_automation_core/maeve_dynamics/pst_reachability.h"

namespace maeve_automation_core {
PST_Reachability::PST_Reachability(PST_Connector&& min_terminal,
                                   PST_Connector&& max_terminal)
    : min_terminal_(std::move(min_terminal)),
      max_terminal_(std::move(max_terminal)) {}

boost::optional<PST_Reachability> PST_Reachability::compute(
    const Eigen::Vector2d& p1, const Interval& v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

Interval PST_Reachability::reachableInterval(
    const PST_Reachability& reachability) {
  const auto& min_connector = PST_Reachability::minConnector(reachability);
  const auto& max_connector = PST_Reachability::maxConnector(reachability);
  const auto min_speed = PST_Connector::terminalSpeed(min_connector);
  const auto max_speed = PST_Connector::terminalSpeed(max_connector);

  return Interval(min_speed, max_speed);
}

const PST_Connector& PST_Reachability::minConnector(
    const PST_Reachability& reachability) {
  return reachability.min_terminal_;
}

const PST_Connector& PST_Reachability::maxConnector(
    const PST_Reachability& reachability) {
  return reachability.max_terminal_;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::I>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::II>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::III>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::IV>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::V>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VI>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VII>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VIII>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

/***/

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::I>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::II>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::III>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::IV>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::V>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VI>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VII>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VIII>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

/***/

template <>
boost::optional<PST_Connector>
PST_Reachability::maxTerminalSpeed<PST_Reachability::Type::V>(
    const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

// LP+
template <>
boost::optional<PST_Connector>
PST_Reachability::maxTerminalSpeed<PST_Reachability::Type::VII>(
    const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  // Intervals for dynamic bounds.
  const auto& I_dt = IntervalConstraints<2>::boundsS<1>(constraints);
  const auto& I_ddt = IntervalConstraints<2>::boundsS<2>(constraints);

  // Terminal speed is max feasible speed.
  const auto dt = Interval::max(I_dt);

  // Constant max acceleration.
  const auto ddt = Interval::max(I_ddt);

  // Compute P+ portion.
  const auto P_plus = Polynomial::fromPointWithDerivatives(p2, dt, ddt);

  // Compute tangent points of L portion.
  const auto rays = Polynomial::tangentRaysThroughPoint(P_plus, p1);
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
  return PST_Connector({p1.y(), p1.y(), r.y(), p2.y()}, {L, L, P_plus});
}

/***/

template <>
boost::optional<PST_Connector>
PST_Reachability::minTerminalSpeed<PST_Reachability::Type::VI>(
    const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::minTerminalSpeed<PST_Reachability::Type::VIII>(
    const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}
}  // namespace maeve_automation_core
