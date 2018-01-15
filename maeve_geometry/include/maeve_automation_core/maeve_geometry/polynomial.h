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
#include <iostream>
#include <tuple>

#include "Eigen/Core"
#include "boost/optional.hpp"

#include "maeve_automation_core/maeve_geometry/interval.h"

namespace maeve_automation_core {
/**
 * @brief This class defines a functor that evaluates a polynomial.
 *
 * @note The class only supports polynomials up to order 2.
 */
class Polynomial {
 public:
  /**
   * @brief Stream overload for polynomial.
   *
   * This overload prints the coefficients of the polynomial as an ordered set.
   *
   * @note Output precision is fixed inside the function definition.
   *
   * @param os The output stream.
   * @param polynomial The polynomial functor.
   *
   * @return The output stream with 'polynomial' serialized to it.
   */
  friend std::ostream& operator<<(std::ostream& os,
                                  const Polynomial& polynomial);

  /**
   * @brief Constructor: default initialize to invalid values.
   */
  Polynomial();

  /**
   * @brief Constructor: build from specified coefficients.
   *
   * @param a The quadratic coefficient.
   * @param b The linear coefficient.
   * @param c The constant coefficient.
   */
  Polynomial(const double a, const double b, const double c);

  /**
   * @brief Constructor: build a linear polynomial from two points.
   *
   * @note By convention, positive slope points from p1 to p2. The quadratic
   * coefficient will always be exactly zero.
   *
   * @param p1 The first point.
   * @param p2 The second point.
   */
  Polynomial(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);

  /**
   * @brief Operator to evaluate the polynomial at a given value.
   *
   * @param x The domain value to evaluate the polynomial at.
   *
   * @return The value of the polynomial at 'x'.
   */
  double operator()(const double x) const;

  /**
   * @brief Get the unique critical point of the polynomial.
   *
   * @note For polynomials that are constant functions, this method returns a
   * null object because there is no unique critical point.
   *
   * @param polynomial The polynomial.
   *
   * @return A nullable object of the unique critical point of the polynomial,
   * or boost::none if no such point exists.
   */
  static boost::optional<Eigen::Vector2d> uniqueCriticalPoint(
      const Polynomial& polynomial);

  /**
   * @brief Compute and return the domains for which the polynomial first
   * derivative has negative and positive sign.
   *
   * @param polynomial The polynomial.
   *
   * @return An ordered tuple of the domain for negative first derivative and
   * positive first derivative. For polynomials without a unique critical point,
   * a null object is returned.
   */
  static boost::optional<std::tuple<Interval, Interval>> dxSignDomainPartition(
      const Polynomial& polynomial);

  /**
   * @brief Compute the roots of a polynomial.
   *
   * @note This method favors accuracy and numeric stability over speed.
   *
   * @param polynomial The polynomial.
   *
   * @return A tuple of the roots (may be NaN). By convention the roots are
   * ordered such that the first root is not larger than the second.
   */
  static std::tuple<double, double> roots(const Polynomial& polynomial);

  /**
   * @brief First derivative of the polynomial at a given domain value.
   *
   * @param polynomial The polynomial.
   * @param x The domain value.
   *
   * @return The first derivative of 'polynomial' at 'x'.
   */
  static double dx(const Polynomial& polynomial, const double x);

  /**
   * @brief The second derivative of the polynomial.
   *
   * @param polynomial The polynomial.
   *
   * @return The second derivative of 'polynomial'.
   */
  static double ddx(const Polynomial& polynomial);

  /**
   * @brief Access the quadratic coefficient.
   *
   * @param polynomial The polynomial.
   *
   * @return The polynomial coefficient.
   */
  static double a(const Polynomial& polynomial);

  /**
   * @brief Access the linear coefficient.
   *
   * @param polynomial The polynomial.
   *
   * @return The linear coefficient.
   */
  static double b(const Polynomial& polynomial);

  /**
   * @brief Access the constant coefficient.
   *
   * @param polynomial The polynomial
   *
   * @return The constant coefficient.
   */
  static double c(const Polynomial& polynomial);

  /**
   * @brief Utility for capturing all coefficients at once.
   *
   * @param polynomial The polynomial equation.
   *
   * @return A tuple of the coefficients.
   */
  static std::tuple<double, double, double> coefficients(
      const Polynomial& polynomial);

  /**
   * @brief Whether the polynomial is a linear function.
   *
   * @param polynomial The polynomial.
   *
   * @return True if the polynomial is a linear function; otherwise false.
   */
  static bool isLinear(const Polynomial& polynomial);

  /**
   * @brief Whether the polynomial is a quadratic function.
   *
   * @param polynomial The polynomial.
   *
   * @return True if the polynomial is a quadratic function; otherwise false.
   */
  static bool isQuadratic(const Polynomial& polynomial);

  /**
   * @brief Whether the polynomial is a constant function.
   *
   * @param polynomial The polynomial.
   *
   * @return True if the polynomial is a constant function; otherwise false.
   */
  static bool isConstant(const Polynomial& polynomial);

  /**
   * @brief A factory method to compute the polynomial equation from a point it
   * passes through and its derivative at that point.
   *
   * @param p The point the curve passes through of the form (domain, range).
   * @param dx The first derivative at 'p'.
   * @param ddx The second derivative.
   *
   * @return The polynomial that contains 'p' and has derivative 's_dot' at 'p'.
   */
  static Polynomial fromPointWithDerivatives(const Eigen::Vector2d& p,
                                             const double dx, const double ddx);

  /**
   * @brief Find tangent points of rays through 'p_r' tangent to 'polynomial'.
   *
   * @param polynomial The polynomial function to compute a ray from.
   * @param p_r The point the ray should pass through.
   * @param epsilon Precision specifier for performing validity checking.
   *
   * @return The tangent points; if no rays exists, null is returned.
   */
  static boost::optional<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>
  tangentRaysThroughPoint(const Polynomial& polynomial,
                          const Eigen::Vector2d& p_r,
                          const double epsilon = 1e-8);

  /**
   * @brief Whether the polynomial has real-valued coefficients.
   *
   * @param polynomial The polynomial.
   *
   * @return True if all coefficients are real-valued; otherwise false.
   */
  static bool valid(const Polynomial& polynomial);

 private:
  /**
   * @brief Coefficients for the polynomial in canonical form.
   *
   * The polynomial is assumed to be in canonical form ax^2 + bx + c = 0, with
   * 'a'
   * at index 0, 'b' at index 1, and 'c' at index 2.
   */
  std::array<double, 3> coefficients_;

  /**
   * @brief Coefficients for the first derivative of the polynomial.
   */
  std::array<double, 2> dx_coefficients_;
};  // class Polynomial
}  // namespace maeve_automation_core
