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
#include "maeve_automation_core/controller_interface_msgs/command2d_manager.h"

namespace maeve_automation_core {
Command2D_Manager::Command2D_Manager() : is_initialized_(false) {}

Command2D_Manager::Command2D_Manager(ros::NodeHandle& nh,
                                     const std::string& topic) {
  initialize(nh, topic);
}

void Command2D_Manager::initialize(ros::NodeHandle& nh,
                                   const std::string& topic) {
  command2D_sub_ =
      nh.subscribe(topic, 1, &Command2D_Manager::command2D_Callback, this);
  is_initialized_ = true;
}

bool Command2D_Manager::isInitialized() const { return is_initialized_; }

void Command2D_Manager::command2D_Callback(
    const controller_interface_msgs::Command2D::ConstPtr& msg) {
  // ROS_INFO_STREAM("Received " << msg->x << ", " << msg->y);
  if (command_queue_.push(*msg)) {
    // ROS_INFO_STREAM("Pushed: " << msg->x << ", " << msg->y);
  }
}

boost::optional<controller_interface_msgs::Command2D>
Command2D_Manager::mostRecentMsg() {
  // Get most recent message and flag if any found.
  auto found = false;
  while (command_queue_.pop(most_recent_msg_)) {
    found = true;
  }

  // If not found, and lazy publishing is not enabled, return invalid command.
  if (!found && !most_recent_msg_.lazy_publishing) {
    return boost::none;
  }

  // Otherwise, return most recent command.
  return most_recent_msg_;
}
}  // namespace maeve_automation_core
