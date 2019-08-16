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
#include "open_maeve/isp_field/isp_field.h"

namespace open_maeve {

//------------------------------------------------------------------------------

cv::Mat zero_ISP_single_channel(const cv::Size& size, const int type) {
  return cv::Mat::zeros(size, type);
}

//------------------------------------------------------------------------------

cv::Mat zeroISP_Field(const int width, const int height, const int type) {
  return zeroISP_Field(cv::Size(width, height), type);
}

//------------------------------------------------------------------------------

cv::Mat zeroISP_Field(const cv::Size& size, const int type) {
  return cv::Mat::zeros(size, type);
}

//------------------------------------------------------------------------------

cv::Mat oneISP_Field(const int width, const int height, const int type) {
  return oneISP_Field(cv::Size(width, height), type);
}

//------------------------------------------------------------------------------

cv::Mat oneISP_Field(const cv::Size& size, const int type) {
  return cv::Mat::ones(size, type);
}

//------------------------------------------------------------------------------

cv::Rect ISP_ROI(const cv::Mat& ISP, const double kernel_height,
                 const double kernel_horizon) {
  // Compute horizon row: prevent the kernel from exceeding image bounds.
  const auto kernel_pixel_height = static_cast<int>(kernel_height * ISP.rows);
  const auto half_height = kernel_pixel_height / 2;
  const auto horizon_row_raw = static_cast<int>(kernel_horizon * ISP.rows);
  const auto horizon_row =
      std::max(half_height, std::min(horizon_row_raw, ISP.rows - half_height));

  // Set ROI.
  const auto top_left_row = horizon_row - half_height;
  const auto top_left_col = 0;
  cv::Rect ROI =
      cv::Rect(top_left_col, top_left_row, ISP.cols, kernel_pixel_height);

  return ROI;
}

//------------------------------------------------------------------------------

}  // namespace open_maeve
