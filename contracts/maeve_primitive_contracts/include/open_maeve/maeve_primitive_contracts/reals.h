/*
 * Copyright 2019 Maeve Automation
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

#include <cmath>
#include <stdexcept>
#include <type_traits>

template <typename T>
class AffinelyExtendedReal {
 public:
  /** @brief Constructor that allows implicit conversion. */
  AffinelyExtendedReal(T value);

 protected:
  T value;
};  // class Real

template <typename T>
AffinelyExtendedReal<T>::AffinelyExtendedReal(T value) : value(value) {
  static_assert(std::is_floating_point<T>::value,
                "Cannot instantiate class with non-floating point type.");

  if (std::isnan(value)) {
    throw std::domain_error("NaN is not a Real.");
  }
}

//------------------------------------------------------------------------------

template <typename T>
class Real : AffinelyExtendedReal<T> {
 public:
  /** @brief Constructor that allows implicit conversion. */
  Real(T value);
};  // class Real

template <typename T>
Real<T>::Real(T value) : AffinelyExtendedReal<T>(value) {
  if (std::isinf(value)) {
    throw std::domain_error("Infinity is not a Real.");
  }
}

//------------------------------------------------------------------------------
