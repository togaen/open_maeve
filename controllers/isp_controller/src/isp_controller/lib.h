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

#include "maeve_automation_core/isp_field/potential_transforms.h"

namespace maeve_automation_core {
/**
 * @brief Compute a biasing horizon for choosing controls.
 *
 * This horizon is used by the control law to bias direction towards `center'.
 * Potential rewards to the left and right are decayed by the given factors the
 * farther from `center' they are.
 *
 * @param center The center of biasing field where bias is 0.
 * @param width The total width of the biasing field.
 * @param left_decay The left biasing decay factor.
 * @param right_decay The right biasing decay factor.
 *
 * @return A row vector containing biasing factors.
 */
cv::Mat biasHorizon(const int center, const int width, const double left_decay,
                    const double right_decay);

/**
 * @brief For each column, compute safe longitudinal controls \in [-1, 1].
 *
 * @note The range of C_u is expected to be initialized with reverse directions,
 * i.e., such that range_min >= range_max. \TODO(me) This is dumb. Invert the
 * sign on the potential fields so that the mapping is direct.
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
 * @brief Find local extrema on the given control horizon.
 *
 * @pre The horizon 'h' shall have at more than two elements.
 *
 * Positions along 'control_horizon' that are inflection points are marked with
 * 1.0 (concave down) or -1.0 (concave up). The first and last indices are
 * always take the value of their neighbor.
 *
 * @param h The control horizon to find extrema for.
 *
 * @return A row vector mask that indicates inflection points in
 * 'control_horizon'.
 */
cv::Mat computeHorizonExtrema(const cv::Mat& h);
}  // namespace maeve_automation_core
