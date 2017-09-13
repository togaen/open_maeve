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

#include <ros/ros.h>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/optional.hpp>

#include "controller_interface_msgs/Command2D.h"

namespace maeve_automation_core {
/**
 * @brief This class subscribes to, converts, and returns Command2D messages.
 */
class Command2D_Manager {
 public:
  /**
   * @brief Constructor: build and set initialized to false.
   */
  Command2D_Manager();

  /**
   * @brief Constructor: This is intended to piggy-back on another node's node
   * handle.
   *
   * @param nh The node handle of an already constructed node.
   * @param topic The topic name to retrieve Command2D messages from.
   */
  Command2D_Manager(ros::NodeHandle& nh, const std::string& topic);

  /**
   * @brief Explicitly initialize the manager.
   *
   * @param nh The node handle of an already constructed node.
   * @param topic The topic anme to retrieve Command2D messages from.
   */
  void initialize(ros::NodeHandle& nh, const std::string& topic);

  /**
   * @brief Return the most recently recieved Command2D message.
   *
   * If lazy_publishing is enabled and no message has been received since the
   * last call, the previously returned message will be returned again.
   * Otherwise a nullable object is returned.
   *
   * @return
   */
  boost::optional<controller_interface_msgs::Command2D> mostRecentMsg();

  /**
   * @brief Check whether this object was initialized upon construction.
   *
   * @return True if initialized; otherwise false.
   */
  bool isInitialized() const;

 private:
  /**
   * @brief Callback for receiving Command2D messages.
   *
   * @param msg The received message.
   */
  void command2D_Callback(
      const controller_interface_msgs::Command2D::ConstPtr& msg);

  /** @brief Flag for checking whether this manager was initialized. */
  bool is_initialized_;
  /** @brief Use this queue to transfer command messages between callbacks. */
  boost::lockfree::spsc_queue<controller_interface_msgs::Command2D,
                              boost::lockfree::capacity<10>>
      command_queue_;
  /** @brief This is the last received message. */
  controller_interface_msgs::Command2D most_recent_msg_;
  /** @brief Subscribe to the command 2d topic. */
  ros::Subscriber command2D_sub_;
};  // class Command2D_Manager
}  // namespace maeve_automation_core
