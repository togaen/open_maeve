# README #

This package provides a thin abstraction layer and utility macros for
implementing structs to contain parameters for ROS packages. The idea is that
the package maintainer defines a struct to contain all parameters for the node
and then uses the utilities in this package to initialize the struct. An
example below.

Assume a parameter file:

```yaml
# my_params.yaml
string_param:  'foo'
integer_param: 1234
float_param:   1234.5
boolean_param: true
```

A struct definition that corresponds to this paramter file might look like:

```c++
// my_params.h
#pragma once

#include <string>

#include "maeve_automation_core/ros_parameter_loading/params_base.h"

struct MyParams : public ParamsBase {
  std::string string_param;
  int integer_param;
  float float_param;
  bool boolean_param;

  __attribute__((warn_unused_result)) bool load(const ros::NodeHandle& nh) override;
};
```

Note that the member variable names must match exactly the parameter names. The
`__attribute__((warn_unused_result))` is not necessary, but it is useful to
help check that the return value of `load()` is being used.

An implementation of the struct definition might look like:

```c++
// my_params.cpp
#include "path/to/my_params.h"

bool MyParams::load(const ros::NodeHandle& nh) {
  // Try to load all params from ROS parameter server. If any param fails,
  // the function immediately returns false.
  LOAD_PARAM(string_param);
  LOAD_PARAM(integer_param);
  LOAD_PARAM(float_param);
  LOAD_PARAM(boolean_param);

  // All params successfully loaded. Sanity checking can be done if wanted.
  // Below are a few convenience macros defined by this package. If any
  // check fails, the function immediately returns false.
  CHECK_GT(float_param, 0.f);
  CHECK_STRICTLY_POSITIVE(integer_param);
  CHECK_NONEMPTY(string_param);

  // All params loaded, and all checks passed. Return success.
  return true;
}
```

It is intended that the node that owns the parameter object calls `load()` and
checks its return flag to ensure that things loaded properly.
