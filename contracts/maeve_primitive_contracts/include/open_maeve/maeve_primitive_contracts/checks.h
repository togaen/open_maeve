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
#include <string>

template <typename T>
bool is_zero(const T& val) {
  return (static_cast<T>(0) == val);
}

//------------------------------------------------------------------------------

template <typename T>
bool is_strictly_positive(const T& val) {
  return (val > static_cast<T>(0));
}

//------------------------------------------------------------------------------

template <typename T>
bool is_strictly_negative(const T& val) {
  return (val < static_cast<T>(0));
}

//------------------------------------------------------------------------------

template <typename T>
bool is_nonpositive(const T& val) {
  return !is_strictly_positive(val);
}

//------------------------------------------------------------------------------

template <typename T>
bool is_nonnegative(const T& val) {
  return !is_strictly_negative(val);
}

//------------------------------------------------------------------------------

template <typename T>
void throw_if_zero(const T& val) {
  if (is_zero(val)) {
    throw std::domain_error("Zero is not a value value");
  }
}

//------------------------------------------------------------------------------

template <typename T>
void throw_if_not_strictly_positive(const T& val) {
  if (!is_strictly_positive(val)) {
    throw std::domain_error(
        std::string("Value ") + std::to_string(val) +
        std::string(" is not strictly positive (> 0) as required."));
  }
}

//------------------------------------------------------------------------------

template <typename T>
void throw_if_not_strictly_negative(const T& val) {
  if (!is_strictly_negative(val)) {
    throw std::domain_error(
        std::string("Value ") + std::to_string(val) +
        std::string(" is not strictly negative (< 0) as required."));
  }
}

//------------------------------------------------------------------------------

template <typename T>
void throw_if_not_nonpositive(const T& val) {
  if (!is_nonpositive(val)) {
    throw std::domain_error(
        std::string("Value ") + std::to_string(val) +
        std::string(" is not non-positive (<= 0) as required."));
  }
}

//------------------------------------------------------------------------------

template <typename T>
void throw_if_not_nonnegative(const T& val) {
  if (!is_nonnegative(val)) {
    throw std::domain_error(
        std::string("Value ") + std::to_string(val) +
        std::string(" is not non-negative (>= 0) as required."));
  }
}

//------------------------------------------------------------------------------

template <typename T>
void throw_if_nan(const T& val) {
  if (std::isnan(val)) {
    throw std::domain_error("NaN is not a valid value.");
  }
}

//------------------------------------------------------------------------------

template <typename T>
void throw_if_infinite(const T& val) {
  if (std::isinf(val)) {
    throw std::domain_error("Infinity is not a valid value.");
  }
}

//------------------------------------------------------------------------------
