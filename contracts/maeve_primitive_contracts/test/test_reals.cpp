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
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "open_maeve/maeve_primitive_contracts/reals.h"

namespace open_maeve {
namespace {
constexpr auto INF = std::numeric_limits<double>::infinity();
constexpr auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

//------------------------------------------------------------------------------

TEST(Maeve_Primitives_Contracts, affinely_extended) {
  EXPECT_THROW({ const auto tmp = AffinelyExtended<double>(NaN); },
               std::domain_error);

  EXPECT_NO_THROW({ const auto tmp = AffinelyExtended<double>(INF); });

  EXPECT_NO_THROW({ const auto tmp = AffinelyExtended<double>(0.0); });
}

//------------------------------------------------------------------------------

TEST(Maeve_Primitives_Contracts, real) {
  EXPECT_THROW({ const auto tmp = Real<double>(NaN); }, std::domain_error);

  EXPECT_THROW({ const auto tmp = Real<double>(INF); }, std::domain_error);

  EXPECT_NO_THROW({ const auto tmp = Real<double>(0.0); });
}

//------------------------------------------------------------------------------

TEST(Maeve_Primitives_Contracts, non_positive) {
  EXPECT_THROW({ const auto tmp = (NonPositive<Real, double>(NaN)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (NonPositive<Real, double>(INF)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (NonPositive<Real, double>(-INF)); },
               std::domain_error);

  EXPECT_NO_THROW({ const auto tmp = (NonPositive<Real, double>(0.0)); });

  EXPECT_THROW({ const auto tmp = (NonPositive<Real, double>(1.0)); },
               std::domain_error);

  EXPECT_NO_THROW({ const auto tmp = (NonPositive<Real, double>(-1.0)); });

  EXPECT_THROW(
      { const auto tmp = (NonPositive<AffinelyExtended, double>(NaN)); },
      std::domain_error);

  EXPECT_THROW(
      { const auto tmp = (NonPositive<AffinelyExtended, double>(INF)); },
      std::domain_error);

  EXPECT_NO_THROW(
      { const auto tmp = (NonPositive<AffinelyExtended, double>(-INF)); });

  EXPECT_NO_THROW(
      { const auto tmp = (NonPositive<AffinelyExtended, double>(0.0)); });

  EXPECT_NO_THROW(
      { const auto tmp = (NonPositive<AffinelyExtended, double>(-1.0)); });

  EXPECT_THROW(
      { const auto tmp = (NonPositive<AffinelyExtended, double>(1.0)); },
      std::domain_error);
}

//------------------------------------------------------------------------------

TEST(Maeve_Primitives_Contracts, strictly_positive) {
  EXPECT_THROW({ const auto tmp = (StrictlyPositive<Real, double>(NaN)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (StrictlyPositive<Real, double>(-INF)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (StrictlyPositive<Real, double>(INF)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (StrictlyPositive<Real, double>(0.0)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (StrictlyPositive<Real, double>(-1.0)); },
               std::domain_error);

  EXPECT_NO_THROW({ const auto tmp = (StrictlyPositive<Real, double>(1.0)); });

  EXPECT_THROW(
      { const auto tmp = (StrictlyPositive<AffinelyExtended, double>(NaN)); },
      std::domain_error);

  EXPECT_THROW(
      { const auto tmp = (StrictlyPositive<AffinelyExtended, double>(-INF)); },
      std::domain_error);

  EXPECT_NO_THROW(
      { const auto tmp = (StrictlyPositive<AffinelyExtended, double>(INF)); });

  EXPECT_THROW(
      { const auto tmp = (StrictlyPositive<AffinelyExtended, double>(0.0)); },
      std::domain_error);

  EXPECT_THROW(
      { const auto tmp = (StrictlyPositive<AffinelyExtended, double>(-1.0)); },
      std::domain_error);

  EXPECT_NO_THROW(
      { const auto tmp = (StrictlyPositive<AffinelyExtended, double>(1.0)); });
}

//------------------------------------------------------------------------------

TEST(Maeve_Primitives_Contracts, non_negative) {
  EXPECT_THROW({ const auto tmp = (NonNegative<Real, double>(NaN)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (NonNegative<Real, double>(INF)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (NonNegative<Real, double>(-INF)); },
               std::domain_error);

  EXPECT_NO_THROW({ const auto tmp = (NonNegative<Real, double>(0.0)); });

  EXPECT_THROW({ const auto tmp = (NonNegative<Real, double>(-1.0)); },
               std::domain_error);

  EXPECT_NO_THROW({ const auto tmp = (NonNegative<Real, double>(1.0)); });

  EXPECT_THROW(
      { const auto tmp = (NonNegative<AffinelyExtended, double>(NaN)); },
      std::domain_error);

  EXPECT_THROW(
      { const auto tmp = (NonNegative<AffinelyExtended, double>(-INF)); },
      std::domain_error);

  EXPECT_NO_THROW(
      { const auto tmp = (NonNegative<AffinelyExtended, double>(INF)); });

  EXPECT_NO_THROW(
      { const auto tmp = (NonNegative<AffinelyExtended, double>(0.0)); });

  EXPECT_THROW(
      { const auto tmp = (NonNegative<AffinelyExtended, double>(-1.0)); },
      std::domain_error);

  EXPECT_NO_THROW(
      { const auto tmp = (NonNegative<AffinelyExtended, double>(1.0)); });
}

//------------------------------------------------------------------------------

TEST(Maeve_Primitives_Contracts, strictly_negative) {
  EXPECT_THROW({ const auto tmp = (StrictlyNegative<Real, double>(NaN)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (StrictlyNegative<Real, double>(-INF)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (StrictlyNegative<Real, double>(INF)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (StrictlyNegative<Real, double>(0.0)); },
               std::domain_error);

  EXPECT_NO_THROW({ const auto tmp = (StrictlyNegative<Real, double>(-1.0)); });

  EXPECT_THROW({ const auto tmp = (StrictlyNegative<Real, double>(1.0)); },
               std::domain_error);

  EXPECT_THROW(
      { const auto tmp = (StrictlyNegative<AffinelyExtended, double>(NaN)); },
      std::domain_error);

  EXPECT_NO_THROW(
      { const auto tmp = (StrictlyNegative<AffinelyExtended, double>(-INF)); });

  EXPECT_THROW(
      { const auto tmp = (StrictlyNegative<AffinelyExtended, double>(INF)); },
      std::domain_error);

  EXPECT_THROW(
      { const auto tmp = (StrictlyNegative<AffinelyExtended, double>(0.0)); },
      std::domain_error);

  EXPECT_NO_THROW(
      { const auto tmp = (StrictlyNegative<AffinelyExtended, double>(-1.0)); });

  EXPECT_THROW(
      { const auto tmp = (StrictlyNegative<AffinelyExtended, double>(1.0)); },
      std::domain_error);
}

//------------------------------------------------------------------------------

TEST(Maeve_Primitives_Contracts, non_zero) {
  EXPECT_THROW({ const auto tmp = (NonZero<Real, double>(NaN)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (NonZero<Real, double>(-INF)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (NonZero<Real, double>(INF)); },
               std::domain_error);

  EXPECT_THROW({ const auto tmp = (NonZero<Real, double>(0.0)); },
               std::domain_error);

  EXPECT_NO_THROW({ const auto tmp = (NonZero<Real, double>(-1.0)); });

  EXPECT_NO_THROW({ const auto tmp = (NonZero<Real, double>(1.0)); });

  EXPECT_THROW({ const auto tmp = (NonZero<AffinelyExtended, double>(NaN)); },
               std::domain_error);

  EXPECT_NO_THROW(
      { const auto tmp = (NonZero<AffinelyExtended, double>(-INF)); });

  EXPECT_NO_THROW(
      { const auto tmp = (NonZero<AffinelyExtended, double>(INF)); });

  EXPECT_THROW({ const auto tmp = (NonZero<AffinelyExtended, double>(0.0)); },
               std::domain_error);

  EXPECT_NO_THROW(
      { const auto tmp = (NonZero<AffinelyExtended, double>(-1.0)); });

  EXPECT_NO_THROW(
      { const auto tmp = (NonZero<AffinelyExtended, double>(1.0)); });
}

//------------------------------------------------------------------------------

}  // namespace open_maeve
