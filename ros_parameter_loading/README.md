# README #

This package provides a thin abstraction layer and utility macros for
data structures that retrieve and contain parameters for ROS packages. In
C++ the intent is that the package maintainer defines a struct to contain all
parameters for the node and then uses the utilities in this package to
initialize the struct. In Python the NodeParams object is a dictionary that
automatically loads all node relative parameters into a dictionary.

Examples below.

## Assumptions ##

* This package assumes parameters are loaded relative to the name of the node
that is loading them.

## Example Usage ##

For the following examples of usage, assume a parameter file:

```yaml
# my_params.yaml
string_param:       'foo'
float_param:        1234.5
other_string_param: 'bar'
other_float_param:  678.9
scoped_params:
  inner_int_param:  3
  inner_bool_param: false
  grouped_params:
    random_param: 0.03
    grouped_param1: 1.2
    grouped_param2: 3.4
```

### Python ###

A node that reads in params might look like:

```python
import rospy
import ros_parameter_loading

if __name__ == '__main__':
    rospy.init_node('my_node')
    node_params = ros_parameter_loading.NodeParams()
    print node_params.string_param
    print node_params.float_param
    print node_params.other_string_param
    print node_params.other_float_param
    print node_params.scoped_params
    print node_params.scoped_params[inner_int_param]
```
The call to `rospy.init_node()` must come before the call to
`ros_parameter_loading.NodeParams()`. Note that top-level params (e.g.
`string_param` and `float_param`) are loaded as attributes, and that nested
params (e.g. `inner_int_param`) are loaded as dictionary entries.

If the class attempts to load a non-existent paramter, a `KeyError` exception
is thrown.

### C++ ###

A struct definition that corresponds to this parameter file might look like:

```c++
// my_params.h
#pragma once

#include <string>

#include "open_maeve/ros_parameter_loading/ros_parameter_loading.h"

struct MyParams : public ParamsBase {
  struct GroupedParams {
    float grouped_param1;
    float grouped_param1;
  };

  struct ScopedParams {
    int inner_int_param;
    bool inner_bool_param;
    GroupedParams grouped_params;
  };

  struct StructParams {
    std::string other_string_param;
    float other_float_param;
  };

  // Storage for 'random_param'.
  double random_param;

  // Storage for 'string_param'.
  std::string string_param;

  // Storage for 'float_param'.
  float float_param;

  // The parameters in this struct correspond to the parameters in the
  // 'scoped_params' parameter namespace.
  ScopedParams scoped_params;

  // This struct is simply a container for parameters.
  StructParams struct_params;

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
  LOAD_PARAM(float_param);
  LOAD_STRUCT_PARAM(struct_params, other_string_param);
  LOAD_STRUCT_PARAM(struct_params, other_float_param);
  LOAD_NS_PARAM(scoped_params, inner_int_param);
  LOAD_NS_PARAM(scoped_params, inner_bool_param);
  LOAD_NS_PARAM(scoped_params.grouped_params, grouped_param1);
  LOAD_NS_PARAM(scoped_params.grouped_params, grouped_param2);
  LOAD_NS_NAME_PARAM("scoped_params/grouped_params", random_param);

  // All params successfully loaded. Sanity checking can be done if wanted.
  // Below are a few convenience macros defined by this package. See
  // maeve_macros pacakge for full list.
  CHECK_GT(float_param, 0.f);
  CHECK_ODD(scoped_params.inner_int_param);
  CHECK_LE(float_param, 1234.5f);
  CHECK_CONTAINS_CLOSED(float_param, 0.0f, 1234.5f);
  CHECK_STRICTLY_POSITIVE(float_param);
  CHECK_NONEMPTY(struct_params.other_string_param);
  CHECK_EQ(scoped_params.inner_int_param, 3);
  CHECK_EQ(random_param, 0.03);

  // All params loaded, and all checks passed. Return success.
  return true;
}
```

It is intended that the node that owns the parameter object calls `load()` and
checks its return flag to ensure that things loaded properly.
