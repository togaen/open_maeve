# README #

This package contains a dynamics library for computing reachability under
acceleration constraints in a Path-Speed-Time (PST) space. The main components
of the library are described below.

## PST\_Reachability ##
This class contains a data structure and methods for representing and computing
reachability in PST space. Specifically, the methods in this class can determine
whether a given point in the path-time plane can be reached from another given
constraints on speed and acceleration. It also computes the set of speeds
reachable as well as connectors that contain representatitve trajectories that
achieve minimum and maximum reachable speeds.

Reachability is represented internally by two connecting trajectories: one that
connects to the target point at the maximum reachable speed, and one that
connects at the minimum reachable speed. Because the set of reachable speeds is
guaranteed to be convex, just these two trajectories suffice to define the
entire set of reachable speeds. The representation of the connecting
trajectories is described below.

See the "Example Usage" for what a reachability object looks like.

## PST\_Connector ##
This class contains a data structor and methods for representing and
manipulating the connecting trajectories used by the `PST_Reachability` class
for computing reachability. PST connectors are bang-singular-bang (BSB) trajectories
that have exactly three components: an initial extremal acceleration component,
followed by a zero acceleration component, and ending with an extremal
acceleration component.

In PST space, a feasible BSB trajectory is guaranteed to exist
between two points if any feasible trajectory exists between them. For this
reason PST connectors are represented as BSB trajectories. The representation
has two parts: first, a four-element array of switching times that define, in
order, the start time, the transition time from extremal to zero acceleration,
the transition from zero to second extremal acceleration, and the end time. The
second part is an array of three quadratic polynomials that describe the three
parts of the trajectory.

See the "Example Usage" for what a connector object looks like.

## IntervalConstraints ##
This class contains a data structure and methods for
representing and manipulating the dynamic constraints under which to compute
reachability. It descibes time, path, speed, and acceleration bounds as
real-valued intervals. It also contains an epsilon parameter that is used for
approximate floating point comparisons.

The class is capable of representing interval constraints up to arbitrary order,
where the order is given as a template parameter. For the current reachability
implementation, only 2nd-order constraints are supported.


## Example Usage ##

The following example code sets up a PST reachability problem and computes the
reachability object.

```c++
#include "maeve_automation_core/maeve_dynamics/pst_reachability.h"

namespace maeve_automation_core {
  //
  // Construct start and target points (time, path position)
  //

  const Eigen::Vector2d p1(0.0, 0.0);
  const Eigen::Vector2d p2(1.0, 0.5);

  //
  // Define dynamic constraints for reachability computation.
  //

  auto time_bounds = Interval(0.0, 10.0);
  auto path_bounds = Interval(0.0, 10.0);
  auto speed_bounds = Interval(0.0, 50.0);
  auto acceleration_bounds = Interval(-2.0, 2.0);

  //
  // Define zero-centered epsilon bounds for floating point comparisons.
  //

  auto eps_bounds = Interval(-1e-6, 1e-6);
 
  //
  // Build 2nd-order constraint object.
  //
 
  constexpr auto ORDER = 2;
  const auto constraints = IntervalConstraints<ORDER>(
      eps_bounds, t_bounds, {s_bounds, s_dot_bounds, s_ddot_bounds});

  //
  // Define speeds available at start point (p1).
  //

  constexpr auto speed = 1.0;
  const auto initial_speeds = Interval(speed, speed);

  //
  // Compute and display reachability.
  //

  if (const auto reachability =
          PST_Reachability::compute(initial_speeds, p1, p2, constraints)) {
      std::cout << "Starting at time-path point (" << p1.x() << ", " << p1.y()
                << ") with speed " << speed
                << ", the set of achievable speeds at time-path point ("
                << p2.x() << ", " << p2.y() << ") is: "
                << PST_Reachability::reachableInterval(*reachability) << "\n";
  }
}  // namespace maeve_automation_core
```

Running the above code will compute reachability and print
the following to console:

```
Starting at time-path point (0, 0) with speed 1, the set of achievable speeds at time-path point (1, 0.5) is: {"min": 0.00000, "max": 1.73205}

```

The reachability object can also be printed directly to console, and will yield
the below (although it won't be so nicely formatted):

```json
{
  "min_speed_connector": {
    "switching_times": [0, 0.125, 0.875, 1],
    "parabola_coefficients": [
      {"a": -2.00000, "b": 1.00000, "c": -0.00000},
      {"a": 0.00000,  "b": 0.50000, "c": 0.03125},
      {"a": -2.00000, "b": 4.00000, "c": -1.50000}
    ]
  },
  "max_speed_connector": {
    "switching_times": [0, 0.25, 0.566987, 1],
    "parabola_coefficients": [
      {"a": -2.00000, "b": 1.00000,  "c": 0.00000},
      {"a": 0.00000,  "b": 0.00000,  "c": 0.12500},
      {"a": 2.00000,  "b": -2.26795, "c": 0.76795}
    ]
  }
}
```
