# README #

This library provides time queue functionality. At its core this is implemented with [boost::circular\_buffer](http://www.boost.org/doc/libs/1_64_0/doc/html/circular_buffer.html), but has the following additional functionality:

* Interpolation: Accessing a time between two elements will linearly interpolate between them. For this to work, elements must have arithmetic operations defined. This is enforced at compile time.
* Auto-clearing: The user can enforce a minimum time gap between elements; pushing an element into the queue that would violate this time gap triggers the queue to be emptied before the push happens.

## Example Usage ##

Usage of the queue is straightforward. It is constructed with buffer\_size and max\_time\_gap parameters, which govern the size of the queue and the maximum allowable time gap between elements. The class has the following public methods:

* insert(timestamp, element): An element is inserted into the queue and associated with a timestamp (type double).
* get(timetamp): An element is retrieved from the queue according to the given timestamp.
* size(): How many elements are currently in the queue.
* empty(): Whether the queue is empty or not.
* bfd\_dt(timestamp): Approximate the time derivative at timestamp using backward finite differencing.


The insertion function requires that timestamps for elements are strictly increasing time (minimum time gap is > 0.0). If that requirement is violated, insertion will fail and it will return false. If the timestamp to be inserted would result in a time gap between elements larger than that specified during construction, the queue is empted before insertion.

Note that the get() method returns elements using [boost::optional](http://www.boost.org/doc/libs/1_64_0/libs/optional/doc/html/index.html). An example of usage is given below, and other examples are in the unit tests:

```c++
// my_file.cpp
#include "open_maeve/maeve_time_queue/maeve_time_queue.h"

namespace {
namespace mac = open_maeve;
}  // namespace

// Parameters for the queue: its maximum size, and maximum allowable time gap
const auto buffer_size = 3;
const auto max_time_gap = 0.75;

// Construct an instance of the time queue that stores doubles.
mac::MaeveTimeQueue<double> mtq(buffer_size, max_time_gap);

// Insert some measurements.
mtq.insert(0.0, 1.0);
mtq.insert(0.1, 2.0);
mtq.insert(0.2, 1.5);

mtq.empty(); // evaluates to false
mtq.size();  // evaluates to 3

// Compute approximate derivative with backward finite differencing.
mtq.bfd_dt(0.5);  // evaluates to 10 units/second
mtq.bfd_dt(0.15); // evaluates to -5 units/second

// Get some measurements: note the use of boost::optional as a return container.
if (const auto m = mtq.get(0.0)) {
  // *m has the value 1.0
}
if (const auto m = mtq.get(0.15)) {
  // *m has the value 1.75
}

// If the call to get() failes, boost::none is returned.
if (const auto m = mtq.get(5.0)) {
  // This code will not execute.
} else {
  // This code will execute.
}

// The queue loses its oldest value and maintains size 3.
mtq.insert(0.3, 4.0);

// This timestamp exceeds the max time gap, so the queue will now have size 1.
mtq.insert(10.0, 5.0);

// This insertion will fail because it violates monotonicity.
if (mtq.insert(9.0, 6.0)) {
  // This code will not execute.
} else {
  // This code will execute.
}

```

