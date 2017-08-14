/*
 * Copyright 2017 Maeve Automation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#pragma once

#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

#include <tuple>

namespace maeve_automation_core {

template <typename T_Element>
/**
 * @brief A simple interpolating time queue.
 */
class MaeveTimeQueue {
 public:
  /** @brief Convenience typedef for referring to buffer elements. */
  typedef std::tuple<double, T_Element> ElementType;

  /**
   * @brief Constructor: set max time gap and initialize storage.
   *
   * The default version of this constructor builds a zero-capacity queue.
   *
   * @param buffer_size The number of elements to allocate memory for.
   * @param max_time_gap The maximum time gap before clearing the queue.
   */
  explicit MaeveTimeQueue(const int buffer_size = 0,
                          const double max_time_gap = 1.0);

  /**
   * @brief Insert an element into the queue.
   *
   * If the time gap between the given timestamp and the most recent element in
   * the queue exceeds the max time gap, the queue is cleared before isnertion.
   *
   * @param time The timestamp associated with the element being inserted.
   * @param el The element being inserted.
   *
   * @return True if the insertion was successful; otherwise false.
   */
  bool insert(const double time, const T_Element& el);

  /**
   * @brief Retrieve an element from the queue.
   *
   * If the requested time does not exactly match an existing element, perform
   * linear interpolation to compute an element to return. Note that this
   * returns a copy of the element in the queue.
   *
   * @param time The timestamp for which to retrieve an element.
   *
   * @return The element matching the requested timestamp, or boost::none if no
   * element matches.
   */
  boost::optional<T_Element> get(const double time) const;

  /**
   * @brief Approximate the derivative at a time with a backward difference.
   *
   * This function computes a backward difference using the given time and
   * offset 'h', where 'h' is the time to the most recent previous measurement.
   *
   * @param time The query time for the finite difference.
   *
   * @return The time derivative of the element at 'time', or boost::none if the
   * operation fails.
   */
  boost::optional<T_Element> dt(const double time) const;

  /**
   * @brief Return the number of elements in the queue.
   *
   * @return The number of elements in the queue.
   */
  typename boost::circular_buffer<ElementType>::size_type size() const;

  /**
   * @brief Whether the queue is empty or not.
   *
   * @return True if the queue is empty; otherwise false.
   */
  bool empty() const;

 private:
  /**
   * @brief Retrieve a tuple of bounding iterators for a given time.
   *
   * If the given time is found exactly, the iterators will be equal and valid.
   * If the time is not contained in the buffer, the iterators will be equal and
   * invalid.
   *
   * @param time The time for which to retrieve bounding iterators.
   *
   * @return The iterators that bound the given time.
   */
  std::tuple<typename boost::circular_buffer<ElementType>::const_iterator,
             typename boost::circular_buffer<ElementType>::const_iterator>
  getBoundingIterators(const double time) const;

  /**
   * @brief Check the time and clear the queue if necessary.
   *
   * @param time The timestamp of the element that will be inserted.
   *
   * @return True if the preparation succeeds; otherwise false.
   */
  bool prepareQueueForInsertion(const double time);

  /** @brief The maximum time gap to allow between two elements. */
  double max_time_gap_;
  /** @brief The circular buffer that stores queue elements. */
  boost::circular_buffer<ElementType> cb_;
};  // class MaeveTimeQueue

#include "impl/maeve_time_queue.impl.h"

}  // namespace maeve_automation_core
