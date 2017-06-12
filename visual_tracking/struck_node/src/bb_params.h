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

#include "maeve_automation_core/ros_parameter_loading/params_base.h"

namespace maeve_automation_core {

/** Parameter class for a geometric description of a 2D bounding box.*/
struct BoundingBoxParams : public ParamsBase {
  /**
   *  \brief Bounding box specification.
   *  @{
   */
  double bb_x_min;
  double bb_x_max;
  double bb_y_min;
  double bb_y_max;
  /** @} */

  /**
   *  \brief alternative representation; computed after loading above params.
   *  @{
   */
  double x_pos;
  double y_pos;
  double width;
  double height;
  /** @} */

  /**
   * @copydoc ParamsBase::ParamsBase()
   */
  bool load(const ros::NodeHandle& nh) override;
};  // struct BoundingBoxParams

}  // namespace maeve_automation_core
