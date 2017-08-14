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

//
// DO NOT INCLUDE THIS FILE. INCLUDE maeve_time_queue.h INSTEAD.
//

#include <stdexcept>
#include <tuple>

template <typename T_Element>
MaeveTimeQueue<T_Element>::MaeveTimeQueue(const int buffer_size,
                                          const double max_time_gap)
    : cb_(buffer_size), max_time_gap_(max_time_gap) {
  if (max_time_gap <= 0.0) {
    throw std::domain_error("max_time_gap must be greater than 0.0");
  }
}

template <typename T_Element>
bool MaeveTimeQueue<T_Element>::empty() const {
  return cb_.empty();
}

template <typename T_Element>
typename boost::circular_buffer<
    typename MaeveTimeQueue<T_Element>::ElementType>::size_type
MaeveTimeQueue<T_Element>::size() const {
  return cb_.size();
}

template <typename T_Element>
bool MaeveTimeQueue<T_Element>::prepareQueueForInsertion(const double time) {
  // If queue is empty consider time valid.
  if (empty()) {
    return true;
  }

  // Compute gap.
  const auto gap = time - std::get<0>(cb_.back());

  // Not strictly increasing?
  if (gap <= 0.0) {
    return false;
  }

  // If gap too large, clear buffer.
  if (gap >= max_time_gap_) {
    cb_.clear();
  }

  // All okay.
  return true;
}

template <typename T_Element>
bool MaeveTimeQueue<T_Element>::insert(const double time, const T_Element& el) {
  const auto b = prepareQueueForInsertion(time);
  if (b) {
    cb_.push_back(std::make_tuple(time, el));
  }
  return b && !empty();
}

template <typename T_Element>
boost::optional<T_Element> MaeveTimeQueue<T_Element>::dt(
    const double time) const {
  if (size() < 2) {
    return boost::none;
  }

  const auto iterators = getBoundingIterators(time);
  auto it_lb = std::get<0>(iterators);
  auto it_ub = std::get<1>(iterators);

  // Query time not contained in queue.
  if (it_lb == cb_.end()) {
    return boost::none;
  }

  // Exact match found.
  if (it_lb == it_ub) {
    // If we're at the beginning, backward differencing can't work.
    if (it_lb == cb_.begin()) {
      return boost::none;
    }

    // Otherwise, point to previous element.
    --it_lb;
  }

  // Approximate derivative with backward difference.
  const auto h = std::get<0>(*it_ub) - std::get<0>(*it_lb);
  const auto& val_prv = std::get<1>(*it_lb);
  const auto& val_nxt = std::get<1>(*it_ub);
  return (val_nxt - val_prv) / h;
}

template <typename T_Element>
std::tuple<typename boost::circular_buffer<
               typename MaeveTimeQueue<T_Element>::ElementType>::const_iterator,
           typename boost::circular_buffer<
               typename MaeveTimeQueue<T_Element>::ElementType>::const_iterator>
MaeveTimeQueue<T_Element>::getBoundingIterators(const double time) const {
  // Empty queue, nothing to do.
  if (cb_.empty()) {
    return std::make_tuple(cb_.end(), cb_.end());
  }

  // First element > time.
  auto it_ub = std::upper_bound(std::begin(cb_), std::end(cb_), time,
                                [&](const double t, const ElementType& el) {
                                  return t < std::get<0>(el);
                                });

  // First element >= time.
  auto it_lb = std::lower_bound(std::begin(cb_), std::end(cb_), time,
                                [&](const ElementType& el, const double t) {
                                  return std::get<0>(el) < t;
                                });

  // All queue elements less than time.
  if (it_lb == std::end(cb_)) {
    return std::make_tuple(cb_.end(), cb_.end());
  }

  // All queue elements greater than time.
  if (it_ub == std::begin(cb_)) {
    return std::make_tuple(cb_.end(), cb_.end());
  }

  // If now it_ub != it_lb, then it_lb must be an exact match.
  if (it_lb != it_ub) {
    return std::make_tuple(it_lb, it_lb);
  }

  // it_ub always points one element ahead, decrement it_lb to get other bound.
  --it_lb;
  return std::make_tuple(it_lb, it_ub);
}

template <typename T_Element>
boost::optional<T_Element> MaeveTimeQueue<T_Element>::get(
    const double time) const {
  const auto iterators = getBoundingIterators(time);
  const auto& it_lb = std::get<0>(iterators);
  const auto& it_ub = std::get<1>(iterators);

  // Query time not contained in queue.
  if (it_lb == cb_.end()) {
    return boost::none;
  }

  // Exact match found.
  if (it_lb == it_ub) {
    return std::get<1>(*it_lb);
  }

  // Interpolate and return.
  const auto time_prv = std::get<0>(*it_lb);
  const auto time_nxt = std::get<0>(*it_ub);
  const auto& val_prv = std::get<1>(*it_lb);
  const auto& val_nxt = std::get<1>(*it_ub);
  const auto s = (time - time_prv) / (time_nxt - time_prv);
  return val_prv + s * (val_nxt - val_prv);
}
