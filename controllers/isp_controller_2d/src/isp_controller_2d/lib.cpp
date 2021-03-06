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
#include "isp_controller_2d/lib.h"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <limits>
#include <vector>

#include "open_maeve/isp_field/isp_field.h"
#include "open_maeve/maeve_geometry/interval.h"

namespace open_maeve {
namespace {
const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

//------------------------------------------------------------------------------

void printISP(const cv::Mat& ISP) {
  std::cout << "\n>>>\n";
  for (auto i = 0; i < ISP.rows; ++i) {
    for (auto j = 0; j < ISP.cols; ++j) {
      std::cout << ISP.at<cv::Point2d>(i, j) << " ";
    }
    std::cout << "\n";
  }
  std::cout << "<<<\n";
}

//------------------------------------------------------------------------------

double column2Yaw(const cv::Mat& image_plane, const double col,
                  const double f_x, const double p_x) {
  const Interval_d col_interval(0.0, static_cast<double>(image_plane.cols - 1));
  const auto bounded_col = Interval_d::project_to_interval(col_interval, col);
  return std::atan2(p_x - static_cast<double>(bounded_col) - 0.5, f_x);
}

//------------------------------------------------------------------------------

double yaw2Column(const cv::Mat& image_plane, const double yaw,
                  const double f_x, const double p_x) {
  const auto offset = f_x * std::tan(yaw);
  const auto col = p_x - offset;
  const Interval_d col_interval(0.0, static_cast<double>(image_plane.cols - 1));
  return Interval_d::project_to_interval(col_interval, col);
}

//------------------------------------------------------------------------------

cv::Mat yawGuidance(const int center, const int width, const double left_decay,
                    const double right_decay) {
  cv::Mat biasing_horizon = zeroISP_Field(width, 1);

  auto decay = 1.0;
  for (auto i = center; i >= 0; --i) {
    biasing_horizon.at<cv::Point2d>(i) = cv::Point2d(decay, 0.0);
    decay = decay * left_decay;
  }

  decay = 1.0;
  for (auto i = center; i < width; ++i) {
    biasing_horizon.at<cv::Point2d>(i) = cv::Point2d(decay, 0.0);
    decay = decay * right_decay;
  }

  return biasing_horizon;
}

//------------------------------------------------------------------------------

cv::Mat avg_reduce_to_horizon(const cv::Mat& ISP, const cv::Rect& ROI) {
  cv::Mat reduced_ISP;
  cv::reduce(ISP(ROI), reduced_ISP, 0 /* 0: row, 1: column */, CV_REDUCE_AVG);
  return reduced_ISP;
}

//------------------------------------------------------------------------------

cv::Mat erodeHorizon(const cv::Mat& h, const double kernel_width) {
  // Reserve return value.
  cv::Mat eroded_h = zeroISP_Field(h.size());

  const auto kernel_pixel_width = static_cast<int>(kernel_width * h.cols);

  // Compute structuring element.
  cv::Mat structuring_element = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(kernel_pixel_width, 1));

  // Erode.
  cv::erode(h, eroded_h, structuring_element);

  // Done.
  return eroded_h;
}

//------------------------------------------------------------------------------

cv::Mat projectThrottlesToControlSpace(
    const cv::Mat& h, const PotentialTransform<ConstraintType::SOFT>& C_u,
    const double K_P, const double K_D) {
  // Reserve return value.
  cv::Mat controls;
  h.copyTo(controls);

  // Project.
  controls.forEach<cv::Point2d>(
      [&](cv::Point2d& p, const int* position) -> void {
        const auto u = cv::Point2d(p.x * K_P + p.y * K_D, 0.0);
        const auto a_min = C_u.shapeParameters().range_min;
        const auto a_max = C_u(u).x;
        p = cv::Point2d(a_min, a_max);
      });

  // Done.
  return controls;
}

//------------------------------------------------------------------------------

cv::Mat throttleGuidance(const cv::Mat& throttle_h, const cv::Mat& guidance_h) {
  // Convert throttle values to unit intervals.
  cv::Mat unit_throttle_h = 0.5 * (cv::Scalar(1.0f, 1.0f) + throttle_h);

  // Get channel of guidance horizon: it's a potential horizon, so channel 1.
  std::vector<cv::Mat> guidance_channels(guidance_h.channels());
  cv::split(guidance_h, guidance_channels);
  const auto& guidance_channel = guidance_channels[0];

  // Get channel with throttle max values: it's a control horizon, so channel 2.
  std::vector<cv::Mat> throttle_channels(unit_throttle_h.channels());
  cv::split(unit_throttle_h, throttle_channels);
  const auto& throttle_channel = throttle_channels[1];

  // Set up throttle guidance storage.
  cv::Mat guided_throttle_h =
      cv::Scalar(-1.0f, 0.0f) + zeroISP_Field(throttle_h.size());
  std::vector<cv::Mat> guided_throttle_channels(guided_throttle_h.channels());
  cv::split(guided_throttle_h, guided_throttle_channels);
  const auto& guided_throttle_max_channel = guided_throttle_channels[1];

  // Apply guidance field.
  cv::multiply(throttle_channel, guidance_channel, guided_throttle_max_channel);

  // Done.
  cv::Mat final_guided_throttle_h;
  cv::merge(guided_throttle_channels, final_guided_throttle_h);
  return final_guided_throttle_h;
}

//------------------------------------------------------------------------------

int dampedMaxThrottleIndex(const cv::Mat& guided_throttle_h,
                           const double inertia, const int damp_idx) {
  std::vector<cv::Mat> guided_throttle_channels(2);
  cv::split(guided_throttle_h, guided_throttle_channels);

  // Find max of available controls.
  auto min_val = NaN;
  auto max_val = NaN;
  std::array<int, 2> min_idx;
  std::array<int, 2> max_idx;
  cv::minMaxIdx(guided_throttle_channels[1], &min_val, &max_val, min_idx.data(),
                max_idx.data());

  // Perform damping: If potential value at maximum control index does not
  // exceed potential inertia, revert control index to bias column.
  const auto damp_idx_val = guided_throttle_h.at<cv::Point2d>(damp_idx).y;
  if ((damp_idx >= 0) && std::abs(damp_idx_val - max_val) <= inertia) {
    max_idx[1] = damp_idx;
  }

  // Done.
  return max_idx[1];
}

//------------------------------------------------------------------------------

}  // namespace open_maeve
