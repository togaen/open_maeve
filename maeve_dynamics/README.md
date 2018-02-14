# README #

This package contains a dynamics library for computing reachability under
acceleration constraints in a Path-Speed-Time (PST) space. There are three paths
for interaction with the library:

* `PST_Reachability`: This class contains a data structure and methods for
representing and computing reachability in PST space. Specifically, the methods
in this class can determine whether a given point in the path-time plane can
be reached from another given constraints on speed and acceleration. It also
computes the set of speeds reachable as well as connectors that contain
representatitve trajectories that achieve minimum and maximum reachable speeds.
* `PST_Connector`: This class contains a data structor and methods for
representing and manipulating the connectors used by the `PST_Reachability`
class for computing reachability. 
* `IntervalConstraints`: This class contains a data structre and methods for
representing and manipulating the dynamic constraints under which to compute
reachability. It descibes time, path, speed, and acceleration bounds as
real-valued intervals. It also contains an epsilon parameter that is used for
approximate floating point comparisons.

See the unit tests for examples and usage.
