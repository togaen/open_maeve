# README #

This package provides a thin abstraction layer and utility macros for
implementing structs to contain parameters for ROS packages. The idea is that
the package maintainer defines a struct to contain all parameters for the node
and then uses the utilities in this package to initialize the struct. An
example below.

Assume a parameter file:

    string_param:  'foo'
    integer_param: 1234
    float_param:   1234.5
    boolean_param: true

A struct definition that corresponds to this paramter file might look like:
    
    #include "maeve\_automation\_core/ros\_parameter\_loading/params\_base.h"

    #include <string>

    struct MyParams : public ParamsBase {
      std::string string\_param;
      int integer\_param;
      float float\_param;
      bool boolean\_param;

      \_\_attribute\_\_((warn\_unused\_result)) bool load(const ros::NodeHandle& nh) override;
    };

Note that the member variable names must match exactly the parameter names. The
`__attribute__((warn_unused_result))` is not necessary, but it is useful to
help check that the return value of `load()` is being used.

An implementation of the struct definition might look like:

    bool MyParams::load(const ros::NodeHandle& nh) {
      // Try to load all params from ROS parameter server. If any param fails,
      // the function immediately returns false.
      LOAD_PARAM(string\_param);
      LOAD_PARAM(integer\_param);
      LOAD_PARAM(float\_param);
      LOAD_PARAM(boolean\_param);

      // All params successfully loaded. Sanity checking can be done if wanted.
      // Below are a few convenience macros defined by this package. If any
      // check fails, the function immediately returns false.
      CHECK_GT(float\_param, 0.f);
      CHECK_STRICTLY_POSITIVE(integer\_param);
      CHECK_NONEMPTY(string\_param);

      // All params loaded, and all checks passed. Return success.
      return true;
    }

It is intended that the node that owns the parameter object calls `load()` and
checks its return flag to ensure that things loaded properly.
