/*
 * Copyright 2019 Maeve Automation
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

#include <stdexcept>

#include <ros/ros.h>
#include <boost/optional.hpp>
#include <boost/thread/mutex.hpp>

namespace maeve_core {
/**
 * @brief This class subscribes to, converts, and returns Command2D messages.
 *
 * @tparam The base message type being subscribed to. Do not use the *Ptr
 * explicitly; the queue internally uses the pointer typedefs.
 */
template <typename T>
class MessageQueue {
 public:
  typedef typename T::ConstPtr T_ptr;

  /** @brief Allow delayed initialization. */
  MessageQueue() = default;

  /**
   * @brief Constructor: This is intended to piggy-back on another node's node
   * handle.
   *
   * @param nh The node handle of an already constructed node.
   * @param topic The topic name to retrieve messages from.
   */
  MessageQueue(ros::NodeHandle& nh, const std::string& topic);

  /** @brief Return the most recently recieved message. */
  virtual boost::optional<T_ptr> most_recent_msg_ptr();

  /**
   * @brief Return a copy of the most recently recieved message.
   *
   * @note This throws if the queue is empty.
   */
  T most_recent_msg();

  /**
   * @brief Return the most recently recieved message.
   *
   * @note This destroys the local message copy upon returning.
   *
   */
  boost::optional<T_ptr> consume_most_recent_msg_ptr();

  /** @brief Initialize and subscribe to topic. */
  void initialize(ros::NodeHandle& nh, const std::string& topic);

  /**
   * @brief Check whether the queue is empty.
   *
   * @note This is a blocking call.
   */
  bool empty();

 protected:
  /**
   * @brief Callback for receiving messages.
   *
   * @param msg The received message.
   */
  void callback(const T_ptr& msg);

  /** @brief Throw an exception if the subscriber is not subscribed. */
  void throw_if_uninitialized();

  /** @brief Protect access to most recent message. */
  boost::mutex msg_mutex_;
  /** @brief This is the last received message. */
  boost::optional<T_ptr> most_recent_msg_ptr_opt_;
  /** @brief Subscribe to the message topic. */
  ros::Subscriber sub_;
};  // class MessageQueue

//------------------------------------------------------------------------------

template <typename T>
MessageQueue<T>::MessageQueue(ros::NodeHandle& nh, const std::string& topic)
    : most_recent_msg_ptr_opt_(boost::none) {
  initialize(nh, topic);
}

//------------------------------------------------------------------------------

template <typename T>
void MessageQueue<T>::initialize(ros::NodeHandle& nh,
                                 const std::string& topic) {
  sub_ = nh.subscribe(topic, 1, &MessageQueue<T>::callback, this);
}

//------------------------------------------------------------------------------

template <typename T>
void MessageQueue<T>::callback(const T_ptr& msg) {
  boost::mutex::scoped_lock lock(msg_mutex_);
  most_recent_msg_ptr_opt_ = msg;
}

//------------------------------------------------------------------------------

template <typename T>
void MessageQueue<T>::throw_if_uninitialized() {
  if (sub_.getTopic().empty()) {
    throw std::runtime_error(
        "Attempted to retrieve message without intializing.");
  }
}

//------------------------------------------------------------------------------

template <typename T>
boost::optional<typename MessageQueue<T>::T_ptr>
MessageQueue<T>::most_recent_msg_ptr() {
  throw_if_uninitialized();

  boost::mutex::scoped_lock lock(msg_mutex_);
  return most_recent_msg_ptr_opt_;
}

//------------------------------------------------------------------------------

template <typename T>
T MessageQueue<T>::most_recent_msg() {
  throw_if_uninitialized();

  {
    boost::mutex::scoped_lock lock(msg_mutex_);
    if (most_recent_msg_ptr_opt_) {
      return *(*most_recent_msg_ptr_opt_);
    }
  }

  throw std::runtime_error(
      "Attempted to retrieve a message from an empty queue.");
}

//------------------------------------------------------------------------------

template <typename T>
boost::optional<typename MessageQueue<T>::T_ptr>
MessageQueue<T>::consume_most_recent_msg_ptr() {
  throw_if_uninitialized();

  boost::mutex::scoped_lock lock(msg_mutex_);
  const auto msg = most_recent_msg_ptr_opt_;
  most_recent_msg_ptr_opt_ = boost::none;
  return msg;
}

//------------------------------------------------------------------------------

template <typename T>
bool MessageQueue<T>::empty() {
  boost::mutex::scoped_lock lock(msg_mutex_);
  return !static_cast<bool>(most_recent_msg_ptr_opt_);
}

//------------------------------------------------------------------------------

}  // namespace maeve_core
