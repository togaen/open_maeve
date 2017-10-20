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
/**
 * @brief Construct and return a zero-filled ISP field.
 *
 * @param width The width of the desired field.
 * @param height The height of the desired field.
 *
 * @return The zeroed ISP field.
 */
cv::Mat zeroISP_Field(const int width, const int height);

/**
 * @brief Construct and return a zero-filled ISP field.
 *
 * @param size The desired size.
 *
 * @return The zeroed ISP field.
 */
cv::Mat zeroISP_Field(const cv::Size& size);

/**
 * @brief Construct and return a one-filled ISP field.
 *
 * @param width The width of the desired field.
 * @param height The height of the desired field.
 *
 * @return The one-filled ISP field.
 */
cv::Mat oneISP_Field(const int width, const int height);

/**
 * @brief Construct and return a one-filled ISP field.
 *
 * @param size The desired size.
 *
 * @return The one-filled ISP field.
 */
cv::Mat oneISP_Field(const cv::Size& size);

}  // namespace maeve_automation_core