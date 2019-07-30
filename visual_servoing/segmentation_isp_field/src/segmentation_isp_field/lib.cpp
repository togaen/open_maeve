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
#include "segmentation_isp_field/lib.h"

#include "maeve_core/isp_field/isp_field.h"

namespace maeve_core {
cv::Mat extractGuidanceField(const cv::Mat& segmentation,
                             const LabelRange& label_range,
                             const cv::Point2d& potential_value) {
  // Instantiate ISP field.
  auto f = zeroISP_Field(segmentation.size());

  // Get mask.
  cv::Mat mask;
  cv::inRange(segmentation, std::get<0>(label_range), std::get<1>(label_range),
              mask);

  // Fill out field.
  cv::Scalar s(potential_value.x, potential_value.y);
  f.setTo(s, mask);

  // Return field.
  return f;
}
}  // namespace maeve_core
