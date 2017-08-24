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

#include "maeve_automation_core/cisp_controller/cisp_controller.h"

namespace maeve_automation_core {

TEST(CISP_Controller, testControlHorizon) {
  cv::Mat m = cv::Mat(3, 3, CV_64FC2);
  const auto epsilon = 0.00001;

  for (auto i = 0; i < 3; ++i) {
    for (auto j = 0; j < 3; ++j) {
      const auto v1 = static_cast<double>(i * 3 + j);
      const auto v2 = 9.0 + v1;
      m.at<cv::Point2d>(i, j) = cv::Point2d(v1, v2);
    }
  }

  // Expected size?
  cv::Mat h = CISP_Controller::reduceCISP(m);
  EXPECT_EQ(h.rows, 1);
  EXPECT_EQ(h.cols, m.cols);

  // Expected values?
  for (auto i = 0; i < 3; ++i) {
    const auto v = h.at<cv::Point2d>(i);
    EXPECT_NEAR(v.x, 3.0 + i, epsilon);
    EXPECT_NEAR(v.y, 12.0 + i, epsilon);
  }
}

}  // namespace maeve_automation_core

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
