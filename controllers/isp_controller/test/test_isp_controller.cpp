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
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "isp_controller/lib.h"
#include "maeve_automation_core/isp_controller/isp_controller.h"

namespace maeve_automation_core {
namespace {
static const auto epsilon = 0.00001;
cv::Mat dummyMatrix(const int rows, const int cols) {
  cv::Mat m = cv::Mat(rows, cols, CV_64FC2);
  const auto offset = rows * cols;
  for (auto i = 0; i < rows; ++i) {
    for (auto j = 0; j < cols; ++j) {
      const auto v1 = static_cast<double>(i * cols + j);
      const auto v2 = offset + v1;
      m.at<cv::Point2d>(i, j) = cv::Point2d(v1, v2);
    }
  }
  return m;
}
}  // namespace

TEST(ISP_Controller, test) {
  // ISP.
  const auto rows = 5;
  const auto cols = 7;
  const cv::Mat m = 0.1 * dummyMatrix(rows, cols);

  // Shape parameters for control projection onto [-1, 1).
  const auto r_min = 1.0;
  const auto r_max = -1.0;
  const auto a = 0.05;
  const auto b = 0.5;
  const ShapeParameters sp(r_min, r_max, a, b);

  // Parameters for computing throttle and yaw from ISP.
  const auto k_w = 3;
  const auto k_ht = 3;
  const auto k_hr = rows / 2;
  const auto fx = 1.0;
  const auto px = static_cast<double>(cols / 2);
  const auto ld = 1.1;
  const auto rd = 1.2;
  const auto kp = 1.0;
  const auto kd = 1.0;
  const auto pi = epsilon;
  const ISP_Controller::Params p(sp, k_w, k_ht, k_hr, fx, px, ld, rd, kp, kd,
                                 pi);

  // Initial control command.
  const auto throttle = 0.0;
  const auto yaw = 0.0;
  const ISP_Controller::ControlCommand u(throttle, yaw);

  // Build controller.
  ISP_Controller controller(p, u);

  // Desired control.
  ISP_Controller::ControlCommand u_d(1.0, 0.5);

  // Compute SD control.
  const auto u_star = controller.SD_Control(m, u_d);
  EXPECT_NEAR(u_star.throttle, 0.280268, epsilon);
  EXPECT_NEAR(u_star.yaw, -1.35213, epsilon);

  // \TODO(me): Should do more testing.
}

TEST(ISP_Controller, testNearestIntervalPoint) {
  const cv::Point2d interval(-3.0, 1.0);
  EXPECT_EQ(nearestIntervalPoint(interval, -3.0), -3.0);
  EXPECT_EQ(nearestIntervalPoint(interval, 1.0), 1.0);
  EXPECT_EQ(nearestIntervalPoint(interval, -5.2), -3.0);
  EXPECT_EQ(nearestIntervalPoint(interval, 3.1), 1.0);
  EXPECT_EQ(nearestIntervalPoint(interval, 0.73), 0.73);
}

TEST(ISP_Controller, testYawColumnConversions) {
  const auto rows = 1;
  const auto cols = 13;
  cv::Mat m = dummyMatrix(rows, cols);
  const auto focal_length_x = 3.7;
  const auto principal_point_x = static_cast<double>(cols / 2) + 1.3;

  for (auto i = 0; i < cols; ++i) {
    const auto center_x = static_cast<double>(i) + 0.5;
    const auto yaw = std::atan2(center_x - principal_point_x, focal_length_x);
    const auto computed_yaw =
        column2Yaw(m, i, focal_length_x, principal_point_x);
    const auto computed_column =
        yaw2Column(m, yaw, focal_length_x, principal_point_x);
    EXPECT_NEAR(yaw, computed_yaw, epsilon);
    EXPECT_EQ(computed_column, i);
  }
}

TEST(ISP_Controller, testBiasHorizon) {
  const auto left_decay = 0.5;
  const auto right_decay = 0.7;
  const auto width = 7;
  const auto center = 4;

  // Compute bias horizon.
  cv::Mat bias_horizon = yawBias(center, width, left_decay, right_decay);
  ASSERT_EQ(bias_horizon.rows, 1);
  ASSERT_EQ(bias_horizon.cols, width);

  // Ground truth.
  std::vector<double> biases{std::pow(left_decay, 4),
                             std::pow(left_decay, 3),
                             std::pow(left_decay, 2),
                             left_decay,
                             1.0,
                             right_decay,
                             std::pow(right_decay, 2)};

  // Compare.
  ASSERT_EQ(biases.size(), bias_horizon.cols);
  for (auto i = 0; i < biases.size(); ++i) {
    const auto p = bias_horizon.at<cv::Point2d>(i);
    EXPECT_NEAR(p.x, biases[i], epsilon);
  }
}

TEST(ISP_Controller, testSafeControls) {
  const auto rows = 5;
  const auto cols = 7;

  // Image space potential field.
  const cv::Mat ISP = dummyMatrix(rows, cols);

  // Control projection.
  const auto range_min = 1.0;
  const auto range_max = -1.0;
  const auto alpha = 0.05;
  const auto beta = 0.5;
  const auto C_u = PotentialTransform<ConstraintType::SOFT>(
      ShapeParameters(range_min, range_max, alpha, beta));
  EXPECT_TRUE(C_u.shapeParameters().valid(false));

  // Kernel parameters.
  const auto kernel_width = 3;
  const auto kernel_height = 3;
  const auto kernel_horizon = ISP.rows / 2;
  const auto K_P = 1.0;
  const auto K_D = 0.0;

  // Check controls.
  /*
  ISP:
  (0, 35)  (1, 36)  (2, 37)  (3, 38)  (4, 39)  (5, 40)  (6, 41)
  (7, 42)  (8, 43)  (9, 44)  (10, 45) (11, 46) (12, 47) (13, 48)
  (14, 49) (15, 50) (16, 51) (17, 52) (18, 53) (19, 54) (20, 55)
  (21, 56) (22, 57) (23, 58) (24, 59) (25, 60) (26, 61) (27, 62)
  (28, 63) (29, 64) (30, 65) (31, 66) (32, 67) (33, 68) (34, 69)
  */

  // Compute control horizon.
  std::vector<double> reduction{21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0};
  cv::Mat h = controlHorizon(ISP, kernel_height, kernel_horizon);
  ASSERT_EQ(reduction.size(), h.cols);
  ASSERT_EQ(h.rows, 1);
  for (auto i = 0; i < h.cols; ++i) {
    const auto p = h.at<cv::Point2d>(i);
    EXPECT_EQ(p.x, reduction[i]);
  }

  // Dilate control horizon.
  std::vector<double> max_filter{22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 27.0};
  cv::Mat dilated_h = dilateHorizon(h, kernel_width);
  ASSERT_EQ(dilated_h.rows, 1);
  ASSERT_EQ(dilated_h.cols, max_filter.size());
  for (auto i = 0; i < h.cols; ++i) {
    const auto p = dilated_h.at<cv::Point2d>(i);
    EXPECT_EQ(p.x, max_filter[i]);
  }

  // Compute controls.
  std::vector<double> a_max_projections{-0.1257805, -0.1537137, -0.1812607,
                                        -0.2083901, -0.2350732, -0.2612837,
                                        -0.2612837};
  cv::Mat controls = safeControls(dilated_h, C_u, K_P, K_D);
  ASSERT_EQ(controls.rows, 1);
  ASSERT_EQ(controls.cols, ISP.cols);

  for (auto i = 0; i < ISP.cols; ++i) {
    const auto p = controls.at<cv::Point2d>(0, i);
    const auto a_max = p.y;
    EXPECT_NEAR(a_max, a_max_projections[i], epsilon);
  }
}
}  // namespace maeve_automation_core

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
