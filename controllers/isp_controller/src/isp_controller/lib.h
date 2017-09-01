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

#include <tuple>
#include <vector>

namespace maeve_automation_core {
/** @brief Pair of indices that bound a range on 1D array. */
typedef std::tuple<int, int> IndexPair;

/**
 * @brief Find local minima on the given control horizon.
 *
 * For minima that are peaks, both indices in the index pair will be the same.
 * For minima that are plateaux the first index will point to the first
 * element in the plateau and the second index will point to the first index
 * past the plateau.
 *
 * @param control_horizon The control horizon to find minima for.
 *
 * @return The of index pairs corresponding to local minima.
 */
std::vector<IndexPair> computeHorizonMinima(const cv::Mat& control_horizon);

/**
 * @brief Project a ISP field onto the control horizon.
 *
 * @param ISP The ISP field to project.
 *
 * @return The projected control horizon, a 1xISP.cols scalar array.
 */
cv::Mat reduceISP(const cv::Mat& ISP);
}  // namespace maeve_automation_core
