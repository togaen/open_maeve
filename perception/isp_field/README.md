# README #

This package defines a library for handling Image Space Potential (ISP) fields. The fields themselves are represented as plain [OpenCV](http://opencv.org/) data structures, so all algebraic operations are supplied by that library.

Because data structures and operations are all provided by OpenCV, this is a thin library. It only provides the following:

* potential\_transforms.h: Hard and soft constraint potential transform functions implemented as functors.
* tau.h: A method for computing tau estimates from discrete scale measurement sequences.
* visualization.h: A visualization function for projecting an ISP field to a "human interpretable" image.

## Parameters ##

Users of this package are responsible for setting the potential transform
parameters, which are represented with ShapeParameter objects. For convenience,
a ROS interface library is provided that can load shape parameters from the ROS
parameter server. The parameter sets for hard and soft constraint transforms
are described below:

* hard\_constraint\_transform: Shape and constraint parameters for the hard constraint potential transform:
    * translation (default 0.0)
    * alpha (default 1.0)
    * beta (default 1.)
    * range\_min: Values greater than this and less than range\_max map to infinity.
    * range\_max: Values less than this and greater than range\_min map to infinity.
* soft\_constraint\_transform: Shape and constraint parameters for the soft constraint potential transform:
    * translation (default 0.0)
    * alpha (default 1.0)
    * beta (default 1.0)
    * range\_min: Infimum of potential value mapping
    * range\_max: Supremum of potential value mapping


