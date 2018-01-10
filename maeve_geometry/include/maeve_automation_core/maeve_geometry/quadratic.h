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

namespace maeve_automation_core {
/**
 * @brief This class defines a functor that evaluates a quadratic.
 */
class Quadratic {
 public:
  /**
   * @brief Stream overload for quadratic.
   *
   * This overload prints the coefficients of the quadratic as an ordered set.
   *
   * @param os The output stream.
   * @param quadratic The quadratic functor.
   *
   * @return The output stream with 'quadratic' serialized to it.
   */
  friend std::ostream& operator<<(std::ostream& os, const Quadratic& quadratic);

  /**
   * @brief Constructor: default initialize to invalid values.
   */
  Quadratic();

  /**
   * @brief Constructor: build from specified coefficients.
   *
   * @param a The quadratic coefficient.
   * @param b The linear coefficient.
   * @param c The constant coefficient.
   */
  Quadratic(const double a, const double b, const double c);

  /**
   * @brief Operator to evaluate the quadratic at a given value.
   *
   * @param x The domain value to evaluate the quadratic at.
   *
   * @return The value of the quadratic at 'x'.
   */
  double operator()(const double x) const;

  /**
   * @brief Compute the roots of a quadratic.
   *
   * @note This method favors accuracy and numeric stability over speed.
   *
   * @param quadratic The quadratic.
   *
   * @return A tuple of the roots (may be NaN). By convention the roots are
   * ordered such that the first root is not larger than the second.
   */
  static std::tuple<double, double> roots(const Quadratic& quadratic);

  /**
   * @brief First derivative of the quadratic at a given time.
   *
   * @param quadratic The quadratic.
   * @param time The time.
   *
   * @return The first derivative of 'quadratic' at 'time'.
   */
  static double dt(const Quadratic& quadratic, const double time);

  /**
   * @brief The second derivative of the quadratic.
   *
   * @param quadratic The quadratic.
   *
   * @return The second derivative of 'quadratic'.
   */
  static double ddt(const Quadratic& quadratic);

  /**
   * @brief Access the quadratic coefficient.
   *
   * @param quadratic The quadratic.
   *
   * @return The quadratic coefficient.
   */
  static double a(const Quadratic& quadratic);

  /**
   * @brief Access the linear coefficient.
   *
   * @param quadratic The quadratic.
   *
   * @return The linear coefficient.
   */
  static double b(const Quadratic& quadratic);

  /**
   * @brief Access the constant coefficient.
   *
   * @param quadratic The quadratic
   *
   * @return The constant coefficient.
   */
  static double c(const Quadratic& quadratic);

 private:
  /**
   * @brief Coefficients for the quadratic in canonical form.
   *
   * The quadratic is assumed to be in canonical form ax^2 + bx + c = 0, with
   * 'a'
   * at index 0, 'b' at index 1, and 'c' at index 2.
   */
  std::array<double, 3> coefficients_;

  /**
   * @brief Coefficients for the first derivative of the quadratic.
   */
  std::array<double, 2> dt_coefficients_;
};  // class Quadratic
}  // namespace maeve_automation_core
