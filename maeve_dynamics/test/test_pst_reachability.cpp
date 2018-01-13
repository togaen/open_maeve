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
#include <gtest/gtest.h>

#include "maeve_automation_core/maeve_dynamics/pst_reachability.h"

namespace maeve_automation_core {
namespace {
const auto c = IntervalConstraints<2>(
    {0, 10}, {Interval(0, 100), Interval(0, 5), Interval(-4, 4)});
}  // namespace

TEST(Maeve_Dynamics_PST_Reachability, testCompute) {
  {
    const Eigen::Vector2d p0(0, 0);
    const Eigen::Vector2d p1(1, 1);
    const auto v_i = Interval(0, 1);

    const auto r = PST_Reachability::compute(p0, v_i, p1, c);
    EXPECT_FALSE(!r);
  }
}

TEST(Maeve_Dynamics_PST_Reachability, testMaxTerminalSpeedV) {
  {
    const Eigen::Vector2d p0(0, 0);
    const Eigen::Vector2d p1(1, 1);

    const auto r =
        PST_Reachability::maxTerminalSpeed<PST_Reachability::Type::V>(p0, p1,
                                                                      c);
    EXPECT_FALSE(!r);
  }
}

TEST(Maeve_Dynamics_PST_Reachability, testMaxTerminalSpeedVII) {
  {
    const Eigen::Vector2d p0(0, 0);
    const Eigen::Vector2d p1(4, 4);

    const auto r =
        PST_Reachability::maxTerminalSpeed<PST_Reachability::Type::VII>(p0, p1,
                                                                        c);
    EXPECT_FALSE(!r);
  }
}
#if 0
  /**
   * Specializations for fixed initial speed connectors.
   * @{
   */
  template <>
  boost::optional<PST_Connector>
  PST_Reachability::connector<PST_Reachability::Type::I>(
      const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  template <>
  boost::optional<PST_Connector>
  PST_Reachability::connector<PST_Reachability::Type::II>(
      const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  template <>
  boost::optional<PST_Connector>
  PST_Reachability::connector<PST_Reachability::Type::III>(
      const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  template <>
  boost::optional<PST_Connector>
  PST_Reachability::connector<PST_Reachability::Type::IV>(
      const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  template <>
  boost::optional<PST_Connector>
  PST_Reachability::connector<PST_Reachability::Type::V>(
      const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  template <>
  boost::optional<PST_Connector>
  PST_Reachability::connector<PST_Reachability::Type::VI>(
      const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);
  /** @} */

  /**
   * Specializations for connectors with initial speed intervals.
   * @{
   */
  template <>
  boost::optional<PST_Connector>
  PST_Reachability::connector<PST_Reachability::Type::I>(
      const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  template <>
  boost::optional<PST_Connector>
  PST_Reachability::connector<PST_Reachability::Type::II>(
      const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  template <>
  boost::optional<PST_Connector>
  PST_Reachability::connector<PST_Reachability::Type::III>(
      const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  template <>
  boost::optional<PST_Connector>
  PST_Reachability::connector<PST_Reachability::Type::IV>(
      const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  template <>
  boost::optional<PST_Connector>
  PST_Reachability::connector<PST_Reachability::Type::V>(
      const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  template <>
  boost::optional<PST_Connector>
  PST_Reachability::connector<PST_Reachability::Type::VI>(
      const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);
  /** @} */


  /**
   * Specializations for connectors with min terminal speeds and undetermined
   * initial speeds.
   * @{
   */
  template <>
  boost::optional<PST_Connector>
  PST_Reachability::minTerminalSpeed<PST_Reachability::Type::VI>(
      const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  template <>
  boost::optional<PST_Connector>
  PST_Reachability::minTerminalSpeed<PST_Reachability::Type::VIII>(
      const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);
  /** @} */
#endif
}  // namespace maeve_automation_core
