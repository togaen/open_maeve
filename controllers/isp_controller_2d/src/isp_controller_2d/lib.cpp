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

#include "maeve_automation_core/isp_field/isp_field.h"

namespace maeve_automation_core {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

void printHorizon(const cv::Mat& h) {
  std::cout << "\n>>>\n";
  for (auto i = 0; i < h.cols; ++i) {
    const auto p = h.at<cv::Point2d>(i);
    std::cout << "(" << p.x << ", " << p.y << ") ";
  }
  std::cout << "\n<<<\n";
}

double column2Yaw(const cv::Mat& image_plane, const double col,
                  const double f_x, const double p_x) {
  const auto bounded_col =
      projectToInterval(0.0, static_cast<double>(image_plane.cols - 1), col);
  return std::atan2(p_x - static_cast<double>(bounded_col) - 0.5, f_x);
}

double yaw2Column(const cv::Mat& image_plane, const double yaw,
                  const double f_x, const double p_x) {
  const auto offset = f_x * std::tan(yaw);
  const auto col = p_x - offset;
  return projectToInterval(0.0, static_cast<double>(image_plane.cols - 1), col);
}

cv::Mat controlSetGuidance(const cv::Mat& controls) {
  cv::Mat biasing_horizon = zeroISP_Field(1, controls.cols);
  // \TODO(me) Formulate this to behave like a balloon being pushed: suppression
  // of potential values in one area result in proportional expansion of
  // potential values elsewhere.
  return biasing_horizon;
}

cv::Mat throttleGuidance(const double throttle, const int width) {
  return cv::Mat::ones(1, width, CV_64FC2) * throttle;
}

cv::Mat yawGuidance(const int center, const int width, const double left_decay,
                    const double right_decay) {
  cv::Mat biasing_horizon(1, width, CV_64FC2);

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

cv::Mat controlHorizon(const cv::Mat& ISP, const double kernel_height,
                       const double kernel_horizon) {
  // Allocate horizon.
  cv::Mat reduced_ISP;

  // Compute horizon row.
  const auto horizon_row = static_cast<int>(kernel_horizon * ISP.rows);

  const auto kernel_pixel_height = static_cast<int>(kernel_height * ISP.rows);

  // Set ROI.
  auto half_height = kernel_pixel_height / 2;
  auto top_left_row = horizon_row - half_height;
  auto top_left_col = 0;
  cv::Rect ROI =
      cv::Rect(top_left_col, top_left_row, ISP.cols, kernel_pixel_height);
  cv::Mat masked_ISP = ISP(ROI);

  // Reduce to single row.
  cv::reduce(masked_ISP, reduced_ISP, 0 /* 0: row, 1: column */, CV_REDUCE_MIN);

  // Done.
  return reduced_ISP;
}

cv::Mat erodeHorizon(const cv::Mat& h, const double kernel_width) {
  // Reserve return value.
  cv::Mat eroded_h;

  const auto kernel_pixel_width = static_cast<int>(kernel_width * h.cols);

  // Compute structuring element.
  cv::Mat structuring_element = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(kernel_pixel_width, 1));

  // Erode.
  cv::erode(h, eroded_h, structuring_element);

  // Done.
  return eroded_h;
}

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

double projectYawToControlSpace(
    const cv::Mat& h, const PotentialTransform<ConstraintType::SOFT>& C_u,
    const double fx, const double px, const double yaw) {
  const auto yaw_max = column2Yaw(h, 0.5, fx, px);
  const auto yaw_min = column2Yaw(h, static_cast<double>(h.cols) - 0.5, fx, px);
  const auto y =
      projectToRange(yaw, yaw_min, yaw_max, C_u.shapeParameters().range_min,
                     C_u.shapeParameters().range_max);
  return y;
}

int dampedMaxThrottleIndex(const cv::Mat& throttle_h,
                           const cv::Mat& potential_h, const double inertia,
                           const int damp_idx) {
  // Get channel with throttle max values.
  std::vector<cv::Mat> throttle_channels(2);
  cv::split(throttle_h, throttle_channels);

  // Find max of available controls.
  auto dummy_val = NaN;
  auto max_val = NaN;
  std::array<int, 2> dummy_idx;
  std::array<int, 2> max_idx;
  cv::minMaxIdx(throttle_channels[1], &dummy_val, &max_val, dummy_idx.data(),
                max_idx.data());

  // Perform damping: If potential value at maximum control index does not
  // exceed potential inertia, revert control index to bias column.
  const auto index_potential_val = potential_h.at<cv::Point2d>(damp_idx).x;
  if (std::abs(index_potential_val - max_val) <= inertia) {
    max_idx[1] = damp_idx;
  }

  // Done.
  return max_idx[1];
}
}  // namespace maeve_automation_core
