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

namespace maeve_automation_core {
/**
 * @brief This class defines a functor that evaluates a parabola.
 */
class Parabola {
 public:
  /**
   * @brief Stream overload for parabola.
   *
   * This overload prints the coefficients of the parabola as an ordered set.
   *
   * @param os The output stream.
   * @param parabola The parabola functor.
   *
   * @return The output stream with 'parabola' serialized to it.
   */
  friend std::ostream& operator<<(std::ostream& os, const Parabola& parabola);

  /**
   * @brief Constructor: default initialize to invalid values.
   */
  Parabola();

  /**
   * @brief Constructor: build from specified coefficients.
   *
   * @param a The quadratic coefficient.
   * @param b The linear coefficient.
   * @param c The constant coefficient.
   */
  Parabola(const double a, const double b, const double c);

  /**
   * @brief Operator to evaluate the parabola at a given value.
   *
   * @param x The domain value to evaluate the parabola at.
   *
   * @return The value of the parabola at 'x'.
   */
  double operator()(const double x) const;

  /**
   * @brief First derivative of the parabola at a given time.
   *
   * @param parabola The parabola.
   * @param time The time.
   *
   * @return The first derivative of 'parabola' at 'time'.
   */
  static double dt(const Parabola& parabola, const double time);

  /**
   * @brief The second derivative of the parabola.
   *
   * @param parabola The parabola.
   *
   * @return The second derivative of 'parabola'.
   */
  static double ddt(const Parabola& parabola);

  /**
   * @brief Access the quadratic coefficient.
   *
   * @param parabola The parabola.
   *
   * @return The quadratic coefficient.
   */
  static double a(const Parabola& parabola);

  /**
   * @brief Access the linear coefficient.
   *
   * @param parabola The parabola.
   *
   * @return The linear coefficient.
   */
  static double b(const Parabola& parabola);

  /**
   * @brief Access the constant coefficient.
   *
   * @param parabola The parabola
   *
   * @return The constant coefficient.
   */
  static double c(const Parabola& parabola);

 private:
  /**
   * @brief Coefficients for the parabola in canonical form.
   *
   * The parabola is assumed to be in canonical form ax^2 + bx + c = 0, with 'a'
   * at index 0, 'b' at index 1, and 'c' at index 2.
   */
  std::array<double, 3> coefficients_;

  /**
   * @brief Coefficients for the first derivative of the parabola.
   */
  std::array<double, 2> dt_coefficients_;
};  // class Parabola
}  // namespace maeve_automation_core
