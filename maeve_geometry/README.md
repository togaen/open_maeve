# README #

This package contains simple geometry library functionality that is not readily
available elsewhere. Included in this package are data structures and algorithms
for representating and operating on:

* Real number intervals
* Disjoint interval sets
* Axis-aligned Bounding Boxes
* Quadratic polynomials

All objects in the `maeve_geometry` library have ostream overloads that
serialize them into [JSON](https://json.org) format. This is intended to make
it easy to quickly to share data with other languages, such as python or matlab,
especially for generating plots. Note, however, that the serialization loses
precision, so it should not be used for numerical applications.

In addition, methods for approximate comparisons with an absolute epsilon are
included. See the unit tests for usage.
