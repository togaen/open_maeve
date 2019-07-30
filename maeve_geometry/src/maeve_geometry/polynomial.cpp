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
#include "maeve_core/maeve_geometry/polynomial.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "boost/io/ios_state.hpp"

#include "maeve_core/maeve_geometry/comparisons.h"
#include "maeve_core/maeve_geometry/powers.h"

namespace maeve_core {
namespace {
const auto NaN = std::numeric_limits<double>::quiet_NaN();
const auto Inf = std::numeric_limits<double>::infinity();
}  // namespace

//------------------------------------------------------------------------------

Polynomial::Polynomial(const double a, const double b, const double c,
                       const Interval<double>& domain)
    : domain_(domain), coefficients_({a, b, c}) {
  if (!Polynomial::is_y_axis(*this)) {
    if (!Polynomial::valid(*this)) {
      std::stringstream ss;
      ss << "Attempted to construct an invalid polynomial: " << *this;
      throw std::domain_error(ss.str());
    }
  }

  // Entire domain must be real valued.
  const auto min = Interval<double>::min(domain_);
  const auto max = Interval<double>::max(domain_);
  if (!std::isfinite(min) || !std::isfinite(max)) {
    std::stringstream ss;
    ss << "Polynomial domain must be real valued. Given domain is: " << domain_;
    throw std::domain_error(ss.str());
  }
}

//------------------------------------------------------------------------------

Polynomial::Polynomial(const Polynomial& p, const Interval<double>& domain) {
  double a, b, c;
  std::tie(a, b, c) = Polynomial::coefficients(p);
  *this = Polynomial(a, b, c, domain);
}

//------------------------------------------------------------------------------

Polynomial::Polynomial(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                       const Interval<double>& domain) {
  // Allocate coefficients.
  auto a = 0.0;
  auto b = NaN;
  auto c = NaN;

  // Compute coefficient values.
  const Eigen::Vector2d d = (p2 - p1);
  if (d.x() == 0.0) {
    b = std::copysign(Inf, (p2.y() - p1.y()));
  } else {
    b = (d.y() / d.x());
    c = (p2.y() - b * p2.x());
  }

  // Poor man's delegated constructor
  *this = Polynomial(a, b, c, domain);
}

//------------------------------------------------------------------------------

boost::optional<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>
Polynomial::findConstrainedCriticalPoints(const Eigen::Vector2d& p,
                                          const double y_critical,
                                          const double ddx,
                                          const double tolerance) {
  // Attempt to get roots.
  const auto A = ddx;
  const auto B = (-2.0 * ddx * p.x());
  const auto C = (ddx * square(p.x()) + y_critical - p.y());
  double x1_critical, x2_critical;
  if (const auto roots = Polynomial::roots(A, B, C, tolerance)) {
    std::tie(x1_critical, x2_critical) = *roots;
  } else {
    return boost::none;
  }

  // Construct critical points.
  const Eigen::Vector2d p1(x1_critical, y_critical);
  const Eigen::Vector2d p2(x2_critical, y_critical);

  // Done.
  return std::make_tuple(p1, p2);
}

//------------------------------------------------------------------------------

bool Polynomial::valid(const Polynomial& polynomial) {
  double a, b, c;
  std::tie(a, b, c) = Polynomial::coefficients(polynomial);
  const auto all_finite =
      (std::isfinite(a) && std::isfinite(b) && std::isfinite(c));
  return all_finite;
}

//------------------------------------------------------------------------------

bool Polynomial::is_y_axis(const Polynomial& polynomial) {
  double a, b, c;
  std::tie(a, b, c) = Polynomial::coefficients(polynomial);
  return ((a == 0.0) && std::isinf(b) && std::isnan(c));
}

//------------------------------------------------------------------------------

boost::optional<Eigen::Vector2d> Polynomial::uniqueCriticalPoint(
    const Polynomial& polynomial) {
  // Find unique critical point of a quadratic equation.
  if (std::abs(Polynomial::a(polynomial)) > 0.0) {
    const auto x_critical = (-Polynomial::dx_coefficient<1>(polynomial) /
                             Polynomial::dx_coefficient<0>(polynomial));
    try {
      const auto y_critical = polynomial(x_critical);
      return Eigen::Vector2d(x_critical, y_critical);
    } catch (...) {
      // Exception thrown if x_critical is outside poly domain.
      return boost::none;
    }
  }

  // This function is not defined for other polynomials.
  return boost::none;
}

//------------------------------------------------------------------------------

Polynomial Polynomial::from_point_with_derivatives(
    const Eigen::Vector2d& p, const double dx, const double ddx,
    const Interval<double>& domain) {
  const auto a = ddx;
  const auto b = dx - 2.0 * a * p.x();
  const auto c = p.y() + p.x() * (a * p.x() - dx);
  return Polynomial(a, b, c, domain);
}

//------------------------------------------------------------------------------

Eigen::Vector2d Polynomial::quadraticPointAtDerivative(const Polynomial& P,
                                                       const double dx) {
  const auto x = ((dx - Polynomial::b(P)) / (2.0 * Polynomial::a(P)));
  return Eigen::Vector2d(x, P(x));
}

//------------------------------------------------------------------------------

boost::optional<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>
Polynomial::tangentRaysThroughPoint(const Polynomial& polynomial,
                                    const Eigen::Vector2d& p_r,
                                    const double epsilon) {
  // Capture coefficients.
  double a, b, c;
  std::tie(a, b, c) = Polynomial::coefficients(polynomial);

  // Compute solving equation.
  const auto A = a;
  const auto B = -2.0 * a * p_r.x();
  const auto C = -b * p_r.x() + p_r.y() - c;

  // Attempt to find the roots.
  double r1, r2;
  if (const auto roots = Polynomial::roots(A, B, C, epsilon)) {
    std::tie(r1, r2) = *roots;
  } else {
    return boost::none;
  }

  // Compute tangent points.
  Eigen::Vector2d p1(r1, polynomial(r1));
  Eigen::Vector2d p2(r2, polynomial(r2));

  // Done.
  return std::make_tuple(p1, p2);
}

//------------------------------------------------------------------------------

double Polynomial::dx(const Polynomial& polynomial, const double x) {
  return Polynomial::dx_coefficient<0>(polynomial) * x +
         Polynomial::dx_coefficient<1>(polynomial);
}

//------------------------------------------------------------------------------

double Polynomial::ddx(const Polynomial& polynomial) {
  return polynomial.coefficients_[0];
}

//------------------------------------------------------------------------------

double Polynomial::a(const Polynomial& polynomial) {
  return polynomial.coefficients_[0];
}

//------------------------------------------------------------------------------

double Polynomial::b(const Polynomial& polynomial) {
  return polynomial.coefficients_[1];
}

//------------------------------------------------------------------------------

double Polynomial::c(const Polynomial& polynomial) {
  return polynomial.coefficients_[2];
}

//------------------------------------------------------------------------------

double Polynomial::operator()(const double x) const {
  // Enforce domain
  if (!Interval<double>::contains(domain_, x)) {
    std::stringstream ss;
    ss << "Attempted to evaluate polynomial for value \"" << x
       << "\", which is outside the domain " << domain_;
    throw std::domain_error(ss.str());
  }

  return ((x * (Polynomial::a(*this) * x + Polynomial::b(*this))) +
          Polynomial::c(*this));
}

//------------------------------------------------------------------------------

const Interval<double>& Polynomial::get_domain(const Polynomial& p) {
  return p.domain_;
}

//------------------------------------------------------------------------------

bool Polynomial::is_constant(const Polynomial& p) {
  double a, b, c;
  std::tie(a, b, c) = Polynomial::coefficients(p);

  const auto a_is_zero = (a == 0.0);
  const auto b_is_zero = (b == 0.0);
  const auto c_is_not_zero = (c != 0.0);
  return (a_is_zero && b_is_zero && c_is_not_zero);
}

//------------------------------------------------------------------------------

std::tuple<double, double, double> Polynomial::coefficients(
    const Polynomial& polynomial) {
  return std::make_tuple(Polynomial::a(polynomial), Polynomial::b(polynomial),
                         Polynomial::c(polynomial));
}

//------------------------------------------------------------------------------

boost::optional<std::tuple<double, double>> Polynomial::roots(
    const Polynomial& polynomial, const double tolerance) {
  // Enforce pre-condition.
  try {
    // Constructor allows y axis representations, so do an extra valid check.
    // TODO(me): I really don't like that this check is needed. Need to get rid
    // of that special y axis behavior.
    if (!Polynomial::valid(polynomial)) {
      return boost::none;
    }
  } catch (...) {
    return boost::none;
  }
  double a, b, c;
  std::tie(a, b, c) = Polynomial::coefficients(polynomial);

  // Not quadratic: either indeterminate or linear.
  if (a == 0.0) {
    // Indeterminate.
    if (b == 0.0) {
      return boost::none;
    }

    // Linear.
    const auto x = -c / b;
    return std::make_tuple(x, x);
  }

  // Compute the discriminant.
  const auto discriminant = clampToZero((b * b - 4.0 * a * c), tolerance);

  // Roots are complex, the range of this function is real, so no solution.
  if (discriminant < 0.0) {
    return boost::none;
  }

  // Choose the sign that avoids catastrophic cancellation.
  const auto discriminant_root = std::copysign(std::sqrt(discriminant), b);

  // Compute the first root.
  const auto r1 = ((-b - discriminant_root) / (2.0 * a));

  // Compute the second root with Vieta's formula.
  // 'r1' will always be nonzero due to sign choice above.
  // Check for zero discriminant to ensure unique roots are exactly identical.
  const auto r2 = ((discriminant == 0.0) ? r1 : (c / (a * r1)));

  // Check for violation of post condition.
  if (!std::isfinite(r1) || !std::isfinite(r2)) {
    std::stringstream ss;
    ss << "Post condition in quadratic root finder violated: one or both roots "
          "exist but are not real valued: "
       << r1 << ", " << r2
       << ". Check the implementation; it appears to be incorrect.";
    throw std::range_error(ss.str());
  }

  // Order the roots lowest and highest.
  std::tuple<double, double> roots = std::minmax(r1, r2);

  // Done.
  return roots;
}

//------------------------------------------------------------------------------

boost::optional<std::tuple<double, double>> Polynomial::roots(
    const double a, const double b, const double c, const double tolerance) {
  try {
    return Polynomial::roots(Polynomial(a, b, c), tolerance);
  } catch (...) {
    return boost::none;
  }
}

//------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const Polynomial& polynomial) {
  static const auto PRECISION = 5;

  // Restore stream state on exit.
  boost::io::ios_all_saver guard(os);

  // Temporarily set desired flags and precision.
  os.setf(std::ios::fixed, std::ios::floatfield);
  os.precision(PRECISION);

  return os << "{\"a\": " << Polynomial::a(polynomial)
            << ", \"b\": " << Polynomial::b(polynomial)
            << ", \"c\": " << Polynomial::c(polynomial)
            << ", \"domain\": " << Polynomial::get_domain(polynomial) << "}";
}

//------------------------------------------------------------------------------

bool operator==(const Polynomial& p1, const Polynomial& p2) {
  const auto domains_equal = (p1.domain_ == p2.domain_);
  const auto coefficients_equal = (p1.coefficients_ == p2.coefficients_);
  return (domains_equal && coefficients_equal);
}

//------------------------------------------------------------------------------

bool operator!=(const Polynomial& p1, const Polynomial& p2) {
  return !(p1 == p2);
}

//------------------------------------------------------------------------------

bool Polynomial::approx_eq(const Polynomial& P1, const Polynomial& P2,
                           const double epsilon) {
  const auto domains_equal = Interval_d::approx_eq(
      Polynomial::get_domain(P1), Polynomial::get_domain(P2), epsilon);

  double a1, b1, c1;
  std::tie(a1, b1, c1) = Polynomial::coefficients(P1);

  double a2, b2, c2;
  std::tie(a2, b2, c2) = Polynomial::coefficients(P2);

  return (domains_equal && approxEq(a1, a2, epsilon) &&
          approxEq(b1, b2, epsilon) && approxEq(c1, c2, epsilon));
}

//------------------------------------------------------------------------------

template <>
double Polynomial::dx_coefficient<0>(const Polynomial& p) {
  return (2.0 * p.coefficients_[0]);
}

//------------------------------------------------------------------------------

template <>
double Polynomial::dx_coefficient<1>(const Polynomial& p) {
  return p.coefficients_[1];
}

//------------------------------------------------------------------------------

}  // namespace maeve_core
