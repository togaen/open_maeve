# README #

This package provides a set of macros intended for use as validity checks. Usage
of these macros improves readability and helps avoid errors caused by typos.
Additionally, the macros log debugging info on failure.

It is intended that these macros be used in places where failure to pass the
check results in a function immediately returning `false`. Examples below.

## Example Usage ##

```c++
// my_checks.cpp
#include <limits>
#include <string>

#include "maeve_core/maeve_macros/checks.h"

bool valid() {
  // Check the following conditions.
  CHECK_FINITE(3.45);
  CHECK_INFINITE(std::numeric_limits<double>::infinity());
  CHECK_GT(1.f, 0.f);
  CHECK_GE(4, 3);
  CHECK_LT(7, 9);
  CHECK_ODD(3);
  CHECK_EVEN(3 + 1);
  CHECK_CONTAINS_CLOSED(-1.0, -1.0, 4.7);
  CHECK_CONTAINS_OPEN(-0.999, -1.0, 4.7);
  CHECK_STRICTLY_POSITIVE(4);
  CHECK_NONEMPTY(std::string("hello world!"));
  CHECK_NONEMPTY(std::vector<double>(1, 3.2));
  CHECK_NE(3, 4);
  CHECK_EQ(std::string("some string"), std::string("some string"));

  // All checks passed.
  return true;
}
```
