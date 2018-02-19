# README #

This package contains a library for manipulating simple geometric types and
functions. The main components of the library are described below.

All objects in the `maeve_geometry` library have ostream overloads that
serialize them into [JSON](https://json.org) format. This is intended to make
it easy to quickly to share data with other languages, such as python or matlab,
especially for generating plots. Note, however, that the serialization loses
precision, so it should not be used for numerical applications.

## AABB ##
This class contains representation and functionality for manipulating
axis-aligned bounding boxes (AABBs). The AABB generalizes the real interval
to dimensions 2 and higher. In this implementation, the dimension is given as
a template parameter. The implementation contains functionality for computing:

* Extents along a given axis
* Volume
* Emptiness and validity checks
* Convex hull of two AABBs
* Point containment within an AABB
* Intersection of two AABBs

### Example Usage ###

```c++
#include "maeve_automation_core/maeve_geometry/aabb.h"

#include <cassert>
#include <iostream>

namespace maeve_automation_core {
  //
  // Construct a 2D unit AABB.
  //

  const auto aabb = AABB<2>({Interval(0, 1), Interval(2, 3)});

  //
  // Test validity and emptiness.
  //

  assert(AABB<2>::valid(aabb));
  assert(!AABB<2>::empty(aabb));

  //
  // Print some properties.
  //

  std::cout << aabb << "\n";
  // Prints:
  // [{"axis": 0, "bounds": {"min": 0.00000, "max": 1.00000}},
  //  {"axis": 1, "bounds": {"min": 2.00000, "max": 3.00000}}]

  std::cout << AABB<2>::min(aabb, 0) << " " << AABB<2>::min(aabb, 1) << "\n";
  // Prints:
  // 0 2

  std::cout << AABB<2>::volume(aabb) << "\n";
  // Prints:
  // 1
}  // namespace maeve_automation_core
```

## Comparisons ##

This is a simple set of comparison operations primarily for performing
approximate comparisons to within an absolute epsilon bounds. There is
additionally a utility for clamping a value to zero within absolute epsilon
bounds and an XOR operator.

### Example Usage ###

```c++
#include "maeve_automation_core/maeve_geometry/comparisons.h"

#include <iostream>

namespace maeve_automation_core {
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
}  // namespace maeve_automation_core
```
## Disjoint Interval ##

The disjoint interval is a set of intervals where each interval is guaranteed
to be disjoint from all others. The class defines comparison operations as well
as insertion and intersection operations that are guaranteed to maintain
disjointness. In effect, the class generalizes 1D intervals to be non-convex.

### Example Usage ###

```c++
#include "maeve_automation_core/maeve_geometry/disjoint_interval.h"

#include <iostream>

namespace maeve_automation_core {
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

}  // namespace maeve_automation_core
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
#include "maeve_automation_core/maeve_geometry/interval.h"

#include <iostream>

namespace maeve_automation_core {
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
}  // namespace maeve_automation_core
```

## Powers ##

This header just contains convenience functions for raising values to integer
powers. They are useful for raising expressions to powers.
