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

#include <exception>
#include <tuple>

namespace maeve_automation_core {

template <typename T_Element>
/**
 * @brief A simple interpolating time queue.
 */
class MaeveTimeQueue {
 public:
  /**
   * @brief Constructor: set max time gap and initialize storage.
   *
   * @param buffer_size The number of elements to allocate memory for.
   * @param max_time_gap The maximum time gap before clearing the queue.
   */
  MaeveTimeQueue(const int buffer_size, const double max_time_gap);

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
   * linear interpolation to compute an element to return.
   *
   * @param time The timestamp for which to retrieve an element.
   *
   * @return The element matching the requested timestamp, or boost::none if no
   * element matches.
   */
  boost::optional<T_Element> get(const double time) const;

 private:
  /** @brief Convenience typedef for referring to buffer elements. */
  typedef std::tuple<double, T_Element> BufferEl;

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
  boost::circular_buffer<BufferEl> cb_;
};  // class MaeveTimeQueue

#include "impl/maeve_time_queue.impl.h"

}  // namespace maeve_automation_core
