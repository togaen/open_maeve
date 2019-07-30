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
#include "maeve_core/isp_field/ros_interface.h"

#include "maeve_core/ros_parameter_loading/macros.h"

namespace maeve_core {
bool loadShapeParamsROS_Params(const ros::NodeHandle& nh, const std::string& ns,
                               ShapeParameters& shape_params) {
  // Attempt to load params.
  LOAD_NAMED_PARAM(ns + "/translation", shape_params.translation);
  LOAD_NAMED_PARAM(ns + "/range_min", shape_params.range_min);
  LOAD_NAMED_PARAM(ns + "/range_max", shape_params.range_max);
  LOAD_NAMED_PARAM(ns + "/alpha", shape_params.alpha);
  LOAD_NAMED_PARAM(ns + "/beta", shape_params.beta);

  // All good.
  return true;
}

}  // namespace maeve_core
