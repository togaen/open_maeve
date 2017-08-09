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
#pragma once

#include <opencv2/opencv.hpp>

namespace maeve_automation_core {
template <typename T_PotentialTransform>
class ImageSpacePotentialField {
 public:
  /**
   * @brief Constructor: allocate the field and zero fill.
   *
   * @param rows The pixel height of the field.
   * @param cols The pixel width of the field.
   */
  ImageSpacePotentialField(const int rows, const int cols);

 private:
  typedef T_PotentialTransform P_Tx;
  cv::Mat field_;
};  // class ImageSpacePotentialField

template <typename T_PotentialTransform>
ImageSpacePotentialField<T_PotentialTransform>::ImageSpacePotentialField(
    const int rows, const int cols)
    : field_(rows, cols, CV_64F) {}
}  // namespace maeve_automation_core
