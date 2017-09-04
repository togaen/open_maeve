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
#include "isp_controller/lib.h"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <limits>
#include <vector>

namespace maeve_automation_core {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

double nearestIntervalPoint(const cv::Point2d& interval, const double point) {
  if (point < interval.x) {
    return interval.x;
  }
  if (point > interval.y) {
    return interval.y;
  }
  return point;
}

double column2Yaw(const cv::Mat& image_plane, const int col, const double f_x,
                  const double p_x) {
  return std::atan2(static_cast<double>(col) - p_x + 0.5, f_x);
}

int yaw2Column(const cv::Mat& image_plane, const double yaw, const double f_x,
               const double p_x) {
  return static_cast<int>(f_x * std::tan(yaw) + p_x);
}

cv::Mat throttleBias(const cv::Mat& controls) {
  cv::Mat throttle_bias = cv::Mat::ones(1, controls.cols, CV_64FC2);
  // \TODO(me)
  return throttle_bias;
}

cv::Mat yawBias(const int center, const int width, const double left_decay,
                const double right_decay) {
  cv::Mat bias_horizon(1, width, CV_64FC2);

  auto decay = 1.0;
  for (auto i = center; i >= 0; --i) {
    bias_horizon.at<cv::Point2d>(i) = cv::Point2d(decay, 0.0);
    decay = decay * left_decay;
  }

  decay = 1.0;
  for (auto i = center; i < width; ++i) {
    bias_horizon.at<cv::Point2d>(i) = cv::Point2d(decay, 0.0);
    decay = decay * right_decay;
  }

  return bias_horizon;
}

cv::Mat controlHorizon(const cv::Mat& ISP, const double kernel_height,
                       const double kernel_horizon) {
  // Allocate horizon.
  cv::Mat reduced_ISP;

  // Set ROI.
  auto half_height = static_cast<int>(kernel_height) / 2;
  auto top_left_row = kernel_horizon - half_height;
  auto top_left_col = 0;
  cv::Rect ROI = cv::Rect(top_left_col, top_left_row, ISP.cols, kernel_height);
  cv::Mat masked_ISP = ISP(ROI);

  // Reduce to single row.
  cv::reduce(masked_ISP, reduced_ISP, 0 /* 0: row, 1: column */, CV_REDUCE_MIN);

  // Done.
  return reduced_ISP;
}

cv::Mat erodeHorizon(const cv::Mat& h, const double kernel_width) {
  // Reserve return value.
  cv::Mat eroded_h;

  // Compute structuring element.
  cv::Mat structuring_element =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_width, 1));

  // Erode.
  cv::erode(h, eroded_h, structuring_element);

  // Done.
  return eroded_h;
}

cv::Mat safeControls(const cv::Mat& h,
                     const PotentialTransform<ConstraintType::SOFT>& C_u,
                     const double K_P, const double K_D) {
  // Reserve return value.
  cv::Mat controls;
  h.copyTo(controls);

  // Project.
  controls.forEach<cv::Point2d>(
      [&](cv::Point2d& p, const int* position) -> void {
        const auto u = cv::Point2d(p.x * K_P + p.y * K_D, 0.0);
        const auto a_min = C_u.shapeParameters().range_max;
        const auto a_max = C_u(u).x;
        p = cv::Point2d(a_min, a_max);
      });

  // Done.
  return controls;
}
}  // namespace maeve_automation_core
