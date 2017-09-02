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

#include "maeve_automation_core/isp_field/potential_transforms.h"

namespace maeve_automation_core {
/** @brief Pair of indices that bound a range on 1D array. */
typedef std::tuple<int, int> IndexPair;

/**
 * @brief For each column, compute safe longitudinal controls \in [-1, 1].
 *
 * @note The range of C_u is expected to be initialized with reverse directions,
 * i.e., such that range_min >= range_max.
 *
 * This function runs a max filter of the given kernel dimensions along the
 * kernel horizon to compute a max potential tuple <p, \dot{p}> at each column
 * index. The dot product <p, \dot{p}> x <K_P, K_D> is taken as the raw maximum
 * acceptable control value at each column index. These raw values are projected
 * into [-1, 1] using the potential transform C_u.
 *
 * @param ISP The input image space potential field.
 * @param C_u The potential transform for mapping controls onto [r_min, r_max]
 * @param kernel_width The width of the max filter kernel.
 * @param kernel_height The height of the max filter kernel.
 * @param kernel_horizon The horizon line to run the max filter along.
 * @param K_P The proportional gain for computing max control.
 * @param K_D The derivative gain for computing max control.
 *
 * @return A two channel, 1D matrix that, where for each column index of ISP,
 * the pixel value defines a safe control range [r_min, a_max].
 */
cv::Mat safeControls(const cv::Mat& ISP,
                     const PotentialTransform<ConstraintType::SOFT>& C_u,
                     const double kernel_width, const double kernel_height,
                     const double kernel_horizon, const double K_P,
                     const double K_D);

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
