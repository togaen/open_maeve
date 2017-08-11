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
#include <opencv2/opencv.hpp>

#include <limits>
#include <vector>

#include "maeve_automation_core/cisp_field/visualize.h"

namespace maeve_automation_core {
cv::Mat computeISPFieldVisualization(const ImageSpacePotentialField& isp,
                                     const double hard_constraint_scale,
                                     const double soft_constraint_scale) {
  // Split out 0th and 1st order channels.
  cv::Mat channel[2];
  cv::split(isp.field(), channel);

  // Map (0, \infty] to red: threshold everything \in [-\infty, 0] to 0.
  cv::Mat repulsive_forces;
  cv::threshold(channel[0], repulsive_forces, 0.0, 0.0, cv::THRESH_TOZERO);

  // Locate infinite values; only positive infinity may be present in ISP.
  const auto finite_values_mask =
      (repulsive_forces < std::numeric_limits<double>::infinity());

  // Find extremal finite values in array.
  auto min_finite_val = std::numeric_limits<double>::quiet_NaN();
  auto max_finite_val = std::numeric_limits<double>::quiet_NaN();
  cv::minMaxIdx(repulsive_forces, &min_finite_val, &max_finite_val, 0, 0,
                finite_values_mask);

  // Storage for color channels.
  std::vector<cv::Mat> color_channels(3);
  auto& r_channel = color_channels[2];
  auto& g_channel = color_channels[1];
  auto& b_channel = color_channels[0];

  // Scale according to user-defined value range.
  repulsive_forces.convertTo(r_channel, CV_8U, 255.0 / hard_constraint_scale);

  // Map (-\infty, 0) to blue
  cv::Mat attractive_forces;
  cv::threshold(channel[0], attractive_forces, 0.0, 0.0, cv::THRESH_TOZERO_INV);

  // Scale according to user-defined value range.
  attractive_forces.convertTo(b_channel, CV_8U, -255.0 / soft_constraint_scale);

  // Map 0 to black: this is implicit in the construction of the visualization.
  g_channel = cv::Mat::zeros(b_channel.rows, b_channel.cols, CV_8U);

  // Construct visualization image.
  cv::Mat visualization;
  cv::merge(color_channels, visualization);

  // Done.
  return visualization;
}
}  // namespace maeve_automation_core
