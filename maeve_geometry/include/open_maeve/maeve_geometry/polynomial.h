
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

#include "open_maeve/maeve_geometry/interval.h"

namespace open_maeve {
/**
 * @brief This class defines a functor that evaluates a polynomial.
 *
 * TODO(me): make sure all methods respect domain information
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
   * @note Output precision is fixed inside the function definition, and the
   * serialization is JSON compatible.
   *
   * @param os The output stream.
   * @param polynomial The polynomial functor.
   *
   * @return The output stream with 'polynomial' serialized to it.
   */
  friend std::ostream& operator<<(std::ostream& os,
                                  const Polynomial& polynomial);

  /** @brief Comparator overloads @{ */
  friend bool operator==(const Polynomial& p1, const Polynomial& p2);
  friend bool operator!=(const Polynomial& p1, const Polynomial& p2);
  /** @} */

  /**
   * @brief Test whether the two polynomials have coefficients and domains
   * within epsilon of each other.
   */
  static bool approx_eq(const Polynomial& P1, const Polynomial& P2,
                        const double epsilon);

  /** @brief No default initialization. */
  Polynomial() = delete;

  /**
   * @brief Constructor: build from specified coefficients.
   *
   * @post The resulting polynomial is guaranteed to be valid or the y axis.
   *
   * TODO(me): It feels a bit sloppy to allow the y axis in there; allowing it
   * for now because I think quite a few things rely on being able to represent
   * a y axis with this object. All those dependencies need to be updated first.
   *
   * @param a The quadratic coefficient.
   * @param b The linear coefficient.
   * @param c The constant coefficient.
   */
  Polynomial(const double a, const double b, const double c,
             const Interval<double>& domain =
                 Interval<double>::max_representable_reals());

  /** @brief Construct a copy of polynomial 'p' with a new domain. */
  Polynomial(const Polynomial& p, const Interval<double>& domain);

  /**
   * @brief Constructor: build a linear polynomial from two points.
   *
   * @note By convention, positive slope points from p1 to p2. The quadratic
   * coefficient will always be exactly zero.
   *
   * @note Internally this uses the Polynomial(a, b, c) constructor.
   *
   * @param p1 The first point.
   * @param p2 The second point.
   */
  Polynomial(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
             const Interval<double>& domain =
                 Interval<double>::max_representable_reals());

  /**
   * @brief Operator to evaluate the polynomial at a given value.
   *
   * @pre 'x' is real-valued
   *
   * @param x The domain value to evaluate the polynomial at.
   *
   * @return The value of the polynomial at 'x'.
   */
  double operator()(const double x) const;

  /** @brief Access the domain of a given polynomial. */
  static const Interval<double>& get_domain(const Polynomial& p);

  /** @brief Check whether the given polynomial is a constant function. */
  static bool is_constant(const Polynomial& p);

  /**
   * @brief Get the unique critical point of the polynomial.
   *
   * @note For polynomials that are constant functions, this method returns a
   * null object because there is no unique critical point.
   *
   * @param polynomial The polynomial.
   *
   * @return A nullable object of the unique critical point of the polynomial,
   * or boost::none if no such point exists, or if the critical point is outside
   * the polynomial's domain.
   */
  static boost::optional<Eigen::Vector2d> uniqueCriticalPoint(
      const Polynomial& polynomial);

  /**
   * @brief Convenience overload of Polynomial::roots(const Polynomial).
   *
   * @param polynomial The polynomial to find roots for.
   *
   * @return See Polynomial::roots(const Polynomial)
   */
  static boost::optional<std::tuple<double, double>> roots(
      const double a, const double b, const double c, const double tolerance);

  /**
   * @brief Compute the real-valued roots of a polynomial.
   *
   * @note This method favors accuracy and numeric stability over speed.
   *
   * @pre Inputs are real valued.
   *
   * @post If the polynomial has a unique root, the tuple members will be
   * exactly identical.
   *
   * @post If the returned optional is not disengaged, the roots are real
   * valued.
   *
   * @param tolerance An optional tolerance value to use to suppress small
   * magnitude negative discriminants.
   *
   * @return A nullable object containing a tuple of the roots, or null if there
   * are no real roots. By convention the roots are ordered such that the first
   * root is not larger than the second.
   */
  static boost::optional<std::tuple<double, double>> roots(
      const Polynomial& polynomial, const double tolerance);

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
   * @brief Find tangent points of rays through 'p_r' tangent to 'polynomial'.
   *
   * @param polynomial The polynomial function to compute a ray from.
   * @param p_r The point the ray should pass through.
   * @param epsilon Precision specifier for performing validity checking.
   *
   * @return The tangent points; if no rays exists, null is returned.
   *
   * TODO(me): parameter values should not have default values
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
   * @return True if all coefficients are real-valued and the polynomial;
   * otherwise false.
   */
  static bool valid(const Polynomial& polynomial);

  /**
   * @brief Test whether the given Polynomial represents the y axis.
   *
   * @note A y-axis polynomial will fail the valid() check because the y axis is
   * not representable by a valid function.
   */
  static bool is_y_axis(const Polynomial& polynomial);

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
  static Polynomial from_point_with_derivatives(
      const Eigen::Vector2d& p, const double dx, const double ddx,
      const Interval<double>& domain =
          Interval<double>::max_representable_reals());

  /**
   * @brief Compute the point at which a polynomial achieves a derivative value.
   *
   * @pre 'P' must be a quadratic.
   *
   * @param P The polynomial.
   * @param dx The derivative.
   *
   * @return The point along 'P' that achieves first derivative 'dx'.
   */
  static Eigen::Vector2d quadraticPointAtDerivative(const Polynomial& P,
                                                    const double dx);

  /**
   * @brief Find the two critical points that lie on 'y_critical' for
   * polynomials that pass through 'p'.
   *
   * @param p The point the polynomials pass through.
   * @param y_critical The critical line.
   * @param ddx The second derivative of the output polynomials.
   *
   * @return A nullable object with the two critical points solving the system;
   * or null if there is no solution.
   */
  static boost::optional<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>
  findConstrainedCriticalPoints(const Eigen::Vector2d& p,
                                const double y_critical, const double ddx,
                                const double tolerance);

 private:
  /** @brief Specify the domain of this function. */
  Interval<double> domain_;

  /**
   * @brief Coefficients for the polynomial in canonical form.
   *
   * The polynomial is assumed to be in canonical form ax^2 + bx + c = 0, with
   * 'a'
   * at index 0, 'b' at index 1, and 'c' at index 2.
   */
  std::array<double, 3> coefficients_;

  /**
   * @brief Compute the coefficients of the first derivative of the polynomial.
   */
  template <int idx>
  static double dx_coefficient(const Polynomial& p);
};  // class Polynomial

/** @brief Specializations for the first derivative computation. @{ */
template <>
double Polynomial::dx_coefficient<0>(const Polynomial& p);

template <>
double Polynomial::dx_coefficient<1>(const Polynomial& p);
/** @} */

}  // namespace open_maeve
