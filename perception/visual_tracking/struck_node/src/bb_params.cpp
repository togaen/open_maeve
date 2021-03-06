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
#include "./bb_params.h"

namespace open_maeve {

bool BoundingBoxParams::load(const ros::NodeHandle& nh) {
  LOAD_PARAM(bb_x_min);
  LOAD_PARAM(bb_x_max);
  LOAD_PARAM(bb_y_min);
  LOAD_PARAM(bb_y_max);

  CHECK_GT(bb_x_max, bb_x_min);
  CHECK_GT(bb_y_max, bb_y_min);

  // Compute alternate representation.
  width = bb_x_max - bb_x_min;
  height = bb_y_max - bb_y_min;
  x_pos = bb_x_min + (width / 2.0);
  y_pos = bb_y_min + (height / 2.0);

  return true;
}

}  // namespace open_maeve
