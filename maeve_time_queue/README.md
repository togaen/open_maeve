# README #

This library provides time queue functionality. At its core this is implemented as a boost circular buffer, but has the following additional functionality:

* Interpolation: Accessing a time between two elements will linearly interpolate between them. For this to work, elements must have arithmetic operations defined. This is enforced at comile time.
* Auto-clearing: The user can enforce a minimum time gap between elements; pushing an element into the queue that would violate this time gap triggers the queue to be emptied before the push happens.

## Usage ##

* Assumes strictly increasing time (minimum time gap is > 0.0)
* Not thread safe
* Cannot guarantee validity of captured references if the queue is modified after capture
