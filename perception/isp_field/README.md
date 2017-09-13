# README #

This package defines a library for handling Image Space Potential (ISP) fields. The fields themselves are represented as plain [OpenCV](http://opencv.org/) data structures, so all algebraic operations are supplied by that library.

Because data structures and operations are all provided by OpenCV, this is a thin library. It only provides the following:

* potential\_transforms.h: Hard and soft constraint potential transform functions implemented as functors.
* tau.h: A method for computing tau estimates from discrete scale measurement sequences.
* visualization.h: A visualization function for projecting an ISP field to a "human interpretable" image.


