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

#include "checks.h"

#include <type_traits>

template <typename T>
class AffinelyExtended {
 public:
  /** @brief Constructor that allows implicit conversion. */
  AffinelyExtended(T value);

 protected:
  T value;
};  // class AffinelyExtended

template <typename T>
AffinelyExtended<T>::AffinelyExtended(T value) : value(value) {
  static_assert(std::is_floating_point<T>::value,
                "Cannot instantiate a Real with non-floating point type.");
  throw_if_nan(value);
}

//------------------------------------------------------------------------------

template <typename T>
class Real : public AffinelyExtended<T> {
 public:
  /** @brief Constructor that allows implicit converstion. */
  Real(T value);
};  // class Real

template <typename T>
Real<T>::Real(T value) : AffinelyExtended<T>(value) {
  throw_if_infinite(value);
}

//------------------------------------------------------------------------------

template <template <typename> class T_Domain, typename T_Scalar>
class NonPositive : public T_Domain<T_Scalar> {
 public:
  /** @brief Constructor that allows implicit converstion. */
  NonPositive(T_Scalar value);
};  // class Real

template <template <typename> class T_Domain, typename T_Scalar>
NonPositive<T_Domain, T_Scalar>::NonPositive(T_Scalar value)
    : T_Domain<T_Scalar>(value) {
  throw_if_not_nonpositive(value);
}

//------------------------------------------------------------------------------

template <template <typename> class T_Domain, typename T_Scalar>
class NonNegative : public T_Domain<T_Scalar> {
 public:
  /** @brief Constructor that allows implicit converstion. */
  NonNegative(T_Scalar value);
};  // class Real

template <template <typename> class T_Domain, typename T_Scalar>
NonNegative<T_Domain, T_Scalar>::NonNegative(T_Scalar value)
    : T_Domain<T_Scalar>(value) {
  throw_if_not_nonnegative(value);
}

//------------------------------------------------------------------------------

template <template <typename> class T_Domain, typename T_Scalar>
class StrictlyPositive : public T_Domain<T_Scalar> {
 public:
  /** @brief Constructor that allows implicit converstion. */
  StrictlyPositive(T_Scalar value);
};  // class Real

template <template <typename> class T_Domain, typename T_Scalar>
StrictlyPositive<T_Domain, T_Scalar>::StrictlyPositive(T_Scalar value)
    : T_Domain<T_Scalar>(value) {
  throw_if_not_strictly_positive(value);
}

//------------------------------------------------------------------------------

template <template <typename> class T_Domain, typename T_Scalar>
class StrictlyNegative : public T_Domain<T_Scalar> {
 public:
  /** @brief Constructor that allows implicit converstion. */
  StrictlyNegative(T_Scalar value);
};  // class Real

template <template <typename> class T_Domain, typename T_Scalar>
StrictlyNegative<T_Domain, T_Scalar>::StrictlyNegative(T_Scalar value)
    : T_Domain<T_Scalar>(value) {
  throw_if_not_strictly_negative(value);
}

//------------------------------------------------------------------------------

template <template <typename> class T_Domain, typename T_Scalar>
class NonZero : public T_Domain<T_Scalar> {
 public:
  /** @brief Constructor that allows implicit converstion. */
  NonZero(T_Scalar value);
};  // class Real

template <template <typename> class T_Domain, typename T_Scalar>
NonZero<T_Domain, T_Scalar>::NonZero(T_Scalar value)
    : T_Domain<T_Scalar>(value) {
  throw_if_zero(value);
}

//------------------------------------------------------------------------------
