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
 * @brief Given a Point2d that represents a number line interval, find the
 * nearest point in the interval to `point'.
 *
 * @param interval The interval.
 * @param point The point.
 *
 * @return The nearest point in the interval to `point'.
 */
double nearestIntervalPoint(const cv::Point2d& interval, const double point);

/**
 * @brief For a given image plane and focal length compute yaw.
 *
 * This function computes the angular displacement in radians from the center of
 * the image plane to the center of a given image column. By convention,
 * negative yaw corresponds to displacements toward negative x (to the left),
 * and positive yaw to positive x (to the right).
 *
 * @param image_plane The image plane.
 * @param col The column index.
 * @param f_x The focal length (pixels) along x.
 * @param p_x The x-coordinate of the camera principal point.
 *
 * @return The angular displacement between the column at 'col' and the optical
 * axis.
 */
double column2Yaw(const cv::Mat& image_plane, const int col, const double f_x,
                  const double p_x);

/**
 * @brief For a given image plane and angular offset, compute column index.
 *
 * This function compute the column index of the image plane column that
 * contains a ray offset by yaw from the optical axis. By convention, yaw is
 * negative when offset in the direction of negative x (to the left), and
 * positive in the direction of positive x (to the right).
 *
 * @param image_plane The image plane.
 * @param yaw The angular offset from optical axis.
 * @param f_x The focal length (pixels) along x.
 * @param p_x The x-coordinate of the camera principal point.
 *
 * @return The column index of image_plane containing a ray offset from the
 * optical axis by yaw.
 */
int yaw2Column(const cv::Mat& image_plane, const double yaw, const double f_x,
               const double p_x);

/**
 * @brief Compute a biasing horizon based on availability of controls.
 *
 * This horizon is used by the control law to bias direction towards larger sets
 * of available controls.
 *
 * @param controls The availabe set of controls for each ISP column.
 *
 * @return A row vector containing biasing factors.
 */
cv::Mat controlSetBias(const cv::Mat& controls);

/**
 * @brief Compute a throttle guidance horizon.
 *
 * @param throttle The throttle value to guide towards.
 * @param width The width of the guidance horizon.
 *
 * @return A row vector containing guidance factors.
 */
cv::Mat throttleGuidance(const double throttle, const int width);

/**
 * @brief Compute a yaw-based biasing horizon for choosing controls.
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
cv::Mat yawBias(const int center, const int width, const double left_decay,
                const double right_decay);

/**
 * @brief Max reduce the ISP to a single row vector.
 *
 * @param ISP The input image space potential field.
 * @param kernel_height The height of the max filter kernel.
 * @param kernel_horizon The horizon line to run the max filter along.
 *
 * @return A single row vector of width ISP.cols that contains the max of each
 * column in ISP.
 */
cv::Mat controlHorizon(const cv::Mat& ISP, const double kernel_height,
                       const double kernel_horizon);

/**
 * @brief Apply a min filter to a control horizon.
 *
 * This function runs a min filter of the given kernel dimensions along the
 * kernel horizon to compute a min potential tuple <p, \dot{p}> at each
 * column index.
 *
 * @pre h should be a row vector.
 *
 * @param h The control horizon.
 * @param kernel_width The min filter kernel width.
 *
 * @return A row vector that is a min filtered version of 'h'.
 */
cv::Mat erodeHorizon(const cv::Mat& h, const double kernel_width);

/**
 * @brief For each column, compute safe longitudinal controls \in [-1, 1].
 *
 * The dot product <p, \dot{p}> \cdot <K_P, K_D> is taken as the raw
 * maximum acceptable control value at each column index. These raw values are
 * projected into [-1, 1] using the potential transform C_u.
 *
 * @param h The control horizon to compute safe sets for.
 * @param C_u The potential transform for mapping controls onto [r_min, r_max]
 * @param K_P The proportional gain for computing max control.
 * @param K_D The derivative gain for computing max control.
 *
 * @return A two channel, 1D matrix that, where for each column index of ISP,
 * the pixel value defines a safe control range [C_u.range_min, a_max].
 */
cv::Mat safeControls(const cv::Mat& h,
                     const PotentialTransform<ConstraintType::SOFT>& C_u,
                     const double K_P, const double K_D);
}  // namespace maeve_automation_core
