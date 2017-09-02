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

#include <vector>

namespace maeve_automation_core {
cv::Mat safeControls(const cv::Mat& ISP,
                     const PotentialTransform<ConstraintType::SOFT>& C_u,
                     const double kernel_width, const double kernel_height,
                     const double kernel_horizon, const double K_P,
                     const double K_D) {
  // Reserve return value.
  cv::Mat controls;

  // Compute structuring element.
  cv::Mat structuring_element =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_width, 1));

  // Set ROI.
  auto half_height = static_cast<int>(kernel_height) / 2;
  auto top_left_row = kernel_horizon - half_height;
  auto top_left_col = 0;
  cv::Rect ROI = cv::Rect(top_left_col, top_left_row, ISP.cols, kernel_height);
  cv::Mat masked_ISP = ISP(ROI);

  // Reduce to single row.
  cv::Mat reduced_ISP;
  cv::reduce(masked_ISP, reduced_ISP, 0 /* 0: row, 1: column */, CV_REDUCE_MAX);

  // Dilate.
  cv::dilate(reduced_ISP, controls, structuring_element);

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

std::vector<IndexPair> computeHorizonMinima(const cv::Mat& control_horizon) {
  std::vector<IndexPair> index_pairs;

  return index_pairs;
}

cv::Mat reduceISP(const cv::Mat& ISP) {
  cv::Mat control_horizon;

  cv::reduce(ISP, control_horizon, 0 /* 0: row, 1: column */, CV_REDUCE_AVG);

  return control_horizon;
}
}  // namespace maeve_automation_core
