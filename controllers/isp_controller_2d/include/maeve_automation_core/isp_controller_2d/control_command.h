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

#include <iostream>

namespace maeve_automation_core {
/**
 * @brief Container for passing control commands.
 */
struct ControlCommand {
  /** @brief The commanded throttle. */
  double throttle;

  /** @brief The commanded yaw. */
  double yaw;

  /**
   * @brief Whether members are set to valid control command values.
   *
   * @return True if the numbers are meaningful; otherwise false.
   */
  bool valid() const;

  /**
   * @brief Constructor: initialize to invalid values.
   */
  ControlCommand();

  /**
   * @brief Constructor: initialize to explicit values.
   *
   * @param t The throttle command.
   * @param y The yaw command.
   */
  ControlCommand(const double t, const double y);
};  // struct ControlCommand

/**
 * @brief Overload output stream operator for control command.
 *
 * @param o The output stream.
 * @param sp The control command object.
 *
 * @return The stream.
 */
std::ostream& operator<<(std::ostream& o, const ControlCommand& u);
}  // namespace maeve_automation_core
