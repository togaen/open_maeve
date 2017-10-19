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
#include <limits>
#include <vector>

#include "maeve_automation_core/isp_field/visualize.h"

namespace maeve_automation_core {
namespace {
const auto INF = std::numeric_limits<double>::infinity();
}  // namespace

cv::Mat computeHorizonVisualization(const cv::Mat& horizon,
                                    const int horizon_viz_height,
                                    const int window_viz_height) {
  // Convert control horizon to single channel, 8-bit unsigned.
  std::vector<cv::Mat> channels(2);
  cv::split(horizon, channels);
  auto& horizon_sc = channels.front();
  cv::Mat horizon_sc_8u;
  horizon_sc.convertTo(horizon_sc_8u, CV_8U, 255.0);

  // Create viz window.
  cv::Mat viz = cv::Mat::zeros(window_viz_height, horizon_sc_8u.cols, CV_8U);

  // Resize control horizon viz.
  cv::Mat horizon_viz;
  cv::Size horizon_viz_size(horizon_sc_8u.cols,
                            std::min(horizon_viz_height, window_viz_height));
  cv::resize(horizon_sc_8u, horizon_viz, horizon_viz_size, 0.0, 0.0,
             cv::INTER_NEAREST);

  // Get region of interest in viz window.
  const auto window_mid_row = window_viz_height / 2;
  const auto horizon_half_height = horizon_viz.rows / 2;
  const auto top_left_x = 0;
  const auto top_left_y = window_mid_row - horizon_half_height;
  cv::Rect roi(top_left_x, top_left_y, horizon_viz_size.width,
               horizon_viz_size.height);

  // Apply control horizon viz to viz window.
  cv::Mat tmp = viz(roi);
  horizon_viz.copyTo(tmp);

  // Done.
  return viz;
}

cv::Mat computeISPFieldVisualization(const cv::Mat& isp,
                                     const double lower_bound,
                                     const double upper_bound) {
  // Split out 0th and 1st order channels.
  cv::Mat channel[2];
  cv::split(isp, channel);
  const auto& p_channel = channel[0];

  // Get a mask of infinite values.
  cv::Mat inf_mask = (p_channel == -INF);

  // Map [-\infty, 0) to red: threshold everything \in [0, \infty) to 0.
  cv::Mat repulsive_forces;
  cv::threshold(p_channel, repulsive_forces, 0.0, 0.0, cv::THRESH_TOZERO_INV);

  // Set the infinite values to upper bound.
  repulsive_forces.setTo(lower_bound, inf_mask);

  // Storage for color channels.
  std::vector<cv::Mat> color_channels(3);
  auto& r_channel = color_channels[2];
  auto& g_channel = color_channels[1];
  auto& b_channel = color_channels[0];

  // Scale according to user-defined value range.
  repulsive_forces.convertTo(r_channel, CV_8U, 255.0 / lower_bound);

  // Map (0, \infty) to blue: threshold everything \in [-\infty, 0) to 0.
  cv::Mat attractive_forces;
  cv::threshold(p_channel, attractive_forces, 0.0, 0.0, cv::THRESH_TOZERO);

  // Scale according to user-defined value range.
  attractive_forces.convertTo(b_channel, CV_8U, 255.0 / upper_bound);

  // Map 0 to black: this is implicit in the construction of the visualization.
  g_channel = cv::Mat::zeros(b_channel.rows, b_channel.cols, CV_8U);

  // Construct visualization image.
  cv::Mat visualization;
  cv::merge(color_channels, visualization);

  // Done.
  return visualization;
}
}  // namespace maeve_automation_core
