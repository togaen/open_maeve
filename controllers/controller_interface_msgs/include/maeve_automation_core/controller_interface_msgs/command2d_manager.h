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

#include "controller_interface_msgs/Command2D.h"
#include "maeve_automation_core/maeve_message_queue/maeve_message_queue.h"

namespace maeve_automation_core {
/** @brief A message manager for Command2D messages. */
class Command2D_Manager
    : public MessageQueue<controller_interface_msgs::Command2D> {
 public:
  /**
   * @brief This override will return the last recieved message if and only if
   * 'sticky_control' is set and no new message has been received.
   *
   * TODO(me): This should not extend MessageQueue. It should define its own
   * interface and use a MessageQueue internally.
   */
  boost::optional<T_ptr> most_recent_msg_ptr() override;

 private:
  boost::optional<T_ptr> last_msg_;
};  // class Command2D_Manager
}  // namespace maeve_automation_core
