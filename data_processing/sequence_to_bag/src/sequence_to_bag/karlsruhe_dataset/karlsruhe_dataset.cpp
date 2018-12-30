/*
 * Copyright 2018 Maeve Automation
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
#include "sequence_to_bag/karlsruhe_dataset/karlsruhe_dataset.h"

namespace maeve_automation_core {
namespace karlsruhe_dataset {

geometry_msgs::Transform getTransformFromOdomToCamera() {
  geometry_msgs::Vector3 T;
  T.x = 1.6;
  T.y = 0.05;
  T.z = 0.6;
  geometry_msgs::Quaternion R;
  R.w = 0.999999756306074;
  R.x = 0.0;
  R.y = -0.000698131644088;
  R.z = 0.0;
  geometry_msgs::Transform Tx;
  Tx.translation = T;
  Tx.rotation = R;
  return Tx;
}

}  // namespace karlsruhe_dataset
}  // namespace maeve_automation_core
