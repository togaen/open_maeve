# README #

This package contains a library for manipulating simple geometric types and
functions. The main components of the library are described below.

All objects in the `maeve_geometry` library have ostream overloads that
serialize them into [JSON](https://json.org) format. This is intended to make
it easy to quickly to share data with other languages, such as python or matlab,
especially for generating plots. Note, however, that the serialization loses
precision, so it should not be used for numerical applications.

## Comparisons ##

This is a simple set of comparison operations primarily for performing
approximate comparisons to within an absolute epsilon bounds. There is
additionally a utility for clamping a value to zero within absolute epsilon
bounds and an XOR operator.

### Example Usage ###

```c++
#include "maeve_core/maeve_geometry/comparisons.h"

#include <iostream>

namespace maeve_core {
static constexpr auto epsilon = 1e-6;

std::cout << exclusiveOr(true, false) << "\n";
// Prints: true

std::cout << approxEq(4.0, 4.0 + 1e-7, epsilon) << "\n";
// Prints: true

std::cout << approxNe(4.0, 4.0 + 1e-7, epsilon) << "\n";
// Prints: false

std::cout << clampToZero(1e-7, epsilon) << "\n";
// Prints: 0.0

std::cout << approxZero(1e-5, epsilon) << "\n";
// Prints: false

std::cout << approxLt(4.0, 4.0 + 1e-5, epsilon) << "\n";
// Prints: true

std::cout << approxLe(1.0, 1.0, epsilon) << "\n";
// Prints: true

std::cout << approxGt(1.0 + 1e-5, 1.0, epsilon) << "\n";
// Prints: true

std::cout << approxGe(1.0 - 1e-5, 1.0, epsilon) << "\n";
// Prints: false
}  // namespace maeve_core
```
## Disjoint Interval ##

The disjoint interval is a set of intervals where each interval is guaranteed
to be disjoint from all others. The class defines comparison operations as well
as insertion and intersection operations that are guaranteed to maintain
disjointness. In effect, the class generalizes 1D intervals to be non-convex.

### Example Usage ###

```c++
#include "maeve_core/maeve_geometry/disjoint_interval.h"

#include <iostream>

namespace maeve_core {
  //
  // Construct a disjoint interval.
  //

  const auto di =
      DisjointInterval({Interval(0, 1), Interval(2, 3), Interval(4, 5)});

  //
  // Test containment.
  //

  std::cout << DisjointInterval::contains(di, 0.5) << "\n";
  // Prints: true

  std::cout << DisjointInterval::contains(di, 7.5) << "\n";
  // Prints: false

  //
  // Test comparison.
  //

  const auto di1 = DisjointInterval({Interval(0, 1), Interval(2, 3)});

  std::cout << (di == di1) << "\n";
  // Prints: false

  std::cout << (di != di1) << "\n";
  // Prints: true

  //
  // Test insertion.
  //

  DisjointInterval::insert(di1, Interval(4, 5));
  std::cout << (di == di1) << "\n";
  // Prints: true

  //
  // Test intersection.
  //

  const auto di2 = DisjointInterval({Interval(0.5, 2.5)});
  const auto di3 = DisjointInterval::intersect(di, di2);
  std::cout << di3 << "\n";
  // Prints:
  // {{"min": 0.50000, "max": 1.00000}, {"min": 2.00000, "max": 2.50000}} 

}  // namespace maeve_core
```

## Interval ##

The interval is a standard 1D real-valued interval. The class implements a
representation and operations on the interval type. Basic operations and 
accessors are implemented, as well as other common operations. See 'Example
Usage'.

Of note is that the interval operations explicity distinguish between the 
notions of "empty," "valid," and "zero length." The distinction is given
below:

* **empty**: An empty interval is equivalent to an empty set. It contains no
elements. It is a valid interval, but because it is empty, the notion of
length is undefined; the length of an empty interval is *not* zero. Instead,
empty intervals return NaN as their length
* **valid**: A valid interval is one that is either empty, or that has a minimum
bound that is less than or equal to its maximum.
* **zero length**: An interval with zero length is an interval whose bounds are
exactly equal. The length is zero because the measure it contains only a single
point, and points have zero measure. However, because it does contain a single
element, the interval is *not* empty.

### Example Usage ###
```c++
#include "maeve_core/maeve_geometry/interval.h"

#include <iostream>

namespace maeve_core {
  //
  // Construct an interval from 0 to 1
  //

  const auto i = Interval(0.0, 1.0);

  //
  // Test accessors and properties
  //

  std::cout << Interval::min(i) << " " << Interval::max(i) << "\n";  
  // Prints: 0.0 1.0
  
  std::cout << Interval::empty(i) << " " << Interval::length(i) << "\n";
  // Prints: false 1.0

  std::cout << Interval::contains(i, 0.3) << "\n";
  // Prints: true

  std::cout << Interval::isSubsetEq(Interval(0.2, 0.4), i) << "\n";
  // Prints: true

  //
  // Test operations.
  //

  std::cout << Interval::add(i, Interval(0.5, 0.7)) << "\n";
  // Prints: {"min": 0.5, "max": 1.7}

  std::cout << Interval::intersect(i, Interval(-1.0, 0.3)) << "\n";
  // Prints: {"min": 0.0, "max": 0.3}

  std::cout << Interval::projectToInterval(i, 0.5) << " "
            << Interval::projectToInterval(i, -1.3) << "\n";
  // Prints: 0.5 0.0

  std::cout << Interval::convexHull(i, Interval(2.0, 3.0)) << "\n";
  // Prints: {"min": 0.0, "max": 3.0}

  std::cout << Interval::merge(i, Interval(0.5, 1.5)) << "\n";
  // Prints: {"min": 0.0, "max": 1.5}

  //
  // Distinguish empty/invalid/zero length
  //

  const auto i_empty = Interval();
  const auto i_zero_length = Interval(0.0, 0.0);
  const auto i_invalid = Interval(1.0, -1.0);

  std::cout << Interval::empty(i_empty) << " " << Interval::empty(i_zero_length)
            << " " << Interval::empty(i_invalid) << "\n";
  // Prints: true false false

  std::cout << Interval::zeroLength(i_empty) << " "
            << Interval::zeroLength(i_zero_length) << " "
            << Interval::zeroLength(i_invalid) << "\n";
  // Prints: false true false

  std::cout << Interval::valid(i_empty) << " " << Interval::valid(i_zero_length)
            << " " << Interval::valid(i_invalid) << "\n";
  // Prints: true true false

  //
  // Test factory methods.
  //

  std::cout << Interval::affinelyExtendedReals() << "\n";
  // Prints: {"min": -Inf, "max": Inf}

  std::cout << Interval::maxRepresentableReals() << "\n";
  // Prints: {"min": -DBL_MAX, "max": DBL_MAX}

  std::cout << Interval::nonNegativeReals() << "\n";
  // Prints: {"min": 0.0, "max": DBL_MAX}

  std::cout << Interval::nonPositiveReals() << "\n";
  // Prints: {"min": -DBL_MAX, "max": 0.0}  
}  // namespace maeve_core
```

## Polynomial ##

This class implements representation and operations for first- and
second-order polynomials. Evaluation and derivative operations are implemented
in addition to several geometric operations described below:

* **roots:** This is a numerically stable method for computing the roots of
a polynomial.
* **uniqueCriticalPoint:** This method computes the unique critical point for
quadratic polynomials.
* **tangentRaysThroughPoint:** This method applies to quadratic polynomials.
For a point outside the parabola, up to to two lines pass through the point
tangent to the parabola. This function computes and returns the tangent points
on the parabola for those lines. If only one such line exists the two tangent
points are identical. If no such lines exist, such as for interior points, a
disengaged optional is returned.
* **fromPointWithDerivatives:** This method computes polynomial coefficients
given a point and the first and second derivatives at that point.
* **quadraticPointAtDerivative:** For a quadratic polynomial and derivative
value, this method computes the point at which the the quadratic has a
derivative equal to the given value.
* **findConstrainedCriticalPoints:** For a given second derivative, horizontal
line, and a point not on that line, exactly two parabolas pass through the point
and have critical points on the line. This function computes and returns the
critical points of those parabolas.


### Example Usage ###

```c++
#include "maeve_core/maeve_geometry/polynomial.h"

#include <iostream>

namespace maeve_core {
  //
  // Construct a quadratic.
  //

  const auto q = Polynomial(1.0, 1.0, 1.0);
  std::cout << q << "\n";
  // Prints: {"a": 1.0, "b": 1.0, "c": 1.0}

  //
  // Test properties.
  //

  std::cout << q(1.0) << "\n";
  // Prints: 3.0

  std::cout << Polynomial::dx(q, 1.0) << " " << Polynomial::ddx(q) << "\n";
  // Prints: 3 1 

  std::cout << !Polynomial::roots(q) << "\n";
  // Prints: true

  if (const auto roots = Polynomial::roots(Polynomial(1.0, 2.0, 1.0))) {
    double r1, r2;
    std::tie(r1, r2) = *roots;
    std::cout << r1 << " " << r2 << "\n";
    // Prints: -1 -1
  }

  //
  // Test operations.
  //

  const auto q1 = Polynomial(3.0, 9.0, 5.0);
  const auto cp = Polynomial::uniqueCriticalPoint(q1);
  std::cout << cp.x() << ", " << cp.y() << "\n";
  // Prints: -1.5 -1.75 

  const auto q2 = Polynomial(1.0, 0.0, 1.0);
  const auto p = Eigen::Vector2d(0.0, 0.0);
  if (const auto t = Polynomial::tangentRaysThroughPoint(q2, p)) {
    Eigen::Vector2d p1, p2;
    std::tie(p1, p2) = *t;
    std::cout << p1.x() << ", " << p1.y() << " | " << p2.x() << ", " << p2.y();
    // Prints: -1 2 1 2 
  }
  
  {
    const auto p = Eigen::Vector2d(3.0, 1.0)
    const auto dx = 0.0;
    const auto ddx = 2.0;
    const auto qp = Polynomial::fromPointWithDerivatives(p, dx, ddx);
    std::cout << qp << "\n";
    // Prints: {"a": 2.0, "b": -12.0, "c": 19.0}
  }

  {
    const auto dx = 1.0;
    const auto p = Polynomial::quadraticPointAtDerivative(q, dx);
    std::cout << p.x() << " " << p.y() << "\n";
    // Prints: 0 1
  }

  {
    const auto p = Eigen::Vector2d(2.0, 3.0);
    const auto y_critical = 0.0;
    const auto ddx = 2.0;
    if (const auto points =
            Polynomial::findConstrainedCriticalPoints(p, y_critical, ddx)) {
      Eigen::Vector2d pt1, pt2;
      std::tie(pt1, pt2) = *points;
      std::cout << pt1.x() << " " << pt1.y() << " " << pt2.x() << " " << pt2.y()
                << "\n";
      // Prints: 1 1 3 1
    }
  }
}  // namespace maeve_core
```

## Powers ##

This header just contains convenience functions for raising values to integer
powers. They are useful for raising expressions to powers.

## Tau ##

This library contains some utility functions for computing time to contatct
(tau).
