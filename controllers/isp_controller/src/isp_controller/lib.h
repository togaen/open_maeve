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

#include <algorithm>

#include "maeve_automation_core/isp_field/potential_transforms.h"

namespace maeve_automation_core {
/**
 * @brief Print a horizon matrix to console. Useful for debugging.
 *
 * @pre 'h' shall be a non-empty row vector.
 *
 * @param h The horizon matrix.
 */
void printHorizon(const cv::Mat& h);

/**
 * @brief Given a number line interval, find the nearrest point in the interval
 * to `point'.
 *
 * @pre interval_min shall be <= interval_max.
 *
 * @tparam T The type of number being compared.
 * @param interval_min The minimum of the interval.
 * @param interval_max The maximum of the interval
 * @param point The point.
 *
 * @return The nearest point in the interval to `point'.
 */
template <typename T>
T projectToInterval(const T& interval_min, const T& interval_max,
                    const T& point) {
  return std::min(interval_max, std::max(point, interval_min));
}

/**
 * @brief Project a value 'val' from one range onto another.
 *
 * @tparam T The type of numbers being projected.
 * @param val The value being projected.
 * @param from_range_min The min of the originating range.
 * @param from_range_max The max of the originating range.
 * @param to_range_min The min of the target range.
 * @param to_range_max The max of the target range.
 *
 * @return The value in the target range with the same proportional offset as
 * 'val' in the originating range.
 */
template <typename T>
T projectToRange(const T& val, const T& from_range_min, const T& from_range_max,
                 const T& to_range_min, const T& to_range_max) {
  const auto s = (val - from_range_min) / (from_range_max - from_range_min);
  return to_range_min + s * (to_range_max - to_range_min);
}

/**
 * @brief For a given image plane and focal length compute yaw.
 *
 * This function computes the angular displacement in radians from the center of
 * the image plane to the center of a given image column. By convention,
 * negative yaw corresponds to displacements toward positive camera x (to the
 * right), and positive yaw to negative camera x (to the left).
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
 * contains a ray offset by yaw from the optical axis. Sign convention for yaw
 * is assumed to be that used by column2Yaw.
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
 * @brief Compute a guidance horizon based on availability of controls.
 *
 * This horizon is used by the control law to bias direction towards larger sets
 * of available controls.
 *
 * @param controls The availabe set of controls for each ISP column.
 *
 * @return A row vector containing guidance factors.
 */
cv::Mat controlSetGuidance(const cv::Mat& controls);

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
 * @brief Compute a yaw-based guidance horizon for choosing controls.
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
cv::Mat yawGuidance(const int center, const int width, const double left_decay,
                    const double right_decay);

/**
 * @brief Min reduce the ISP to a single row vector.
 *
 * @pre kernel_height shall correspond to at least one whole pixel.
 *
 * @param ISP The input image space potential field.
 * @param kernel_height Min filter kernel height from 0 - top, to 1 - bottom.
 * @param kernel_horizon Location of horizon line from 0 - top, to 1 - bottom.
 *
 * @return A single row vector of width ISP.cols that contains the min of each
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
 * @pre h should be a row vector and kernel_width shall correspond to at least
 * one whole pixel.
 *
 * @param h The control horizon.
 * @param kernel_width Min filter kernel with from 0 - top, to 1 - bottom.
 *
 * @return A row vector that is a min filtered version of 'h'.
 */
cv::Mat erodeHorizon(const cv::Mat& h, const double kernel_width);

/**
 * @brief For each column, project to throttle controls \in [r_min, r_max].
 *
 * The dot product <p, \dot{p}> \cdot <K_P, K_D> is taken as the raw
 * maximum acceptable control value at each column index. These raw values are
 * projected into [-1, 1] using the potential transform C_u.
 *
 * @param h The control horizon to perform projection on.
 * @param C_u The potential transform for mapping controls onto [r_min, r_max]
 * @param K_P The proportional gain for computing max control.
 * @param K_D The derivative gain for computing max control.
 *
 * @return A two channel, 1D matrix that, where for each column index of ISP,
 * the pixel value defines a safe control range [C_u.range_min, a_max].
 */
cv::Mat projectThrottlesToControlSpace(
    const cv::Mat& h, const PotentialTransform<ConstraintType::SOFT>& C_u,
    const double K_P, const double K_D);

/**
 * @brief Project a given yaw value to [r_min, r_max].
 *
 * @param h The horizon over which the yaw is defined.
 * @param C_u The potential transform for mapping controls onto [r_min, r_max].
 * @param fx The camera focal length (x axis).
 * @param px The camera principal point (x coordinate).
 * @param yaw The yaw value to project.
 *
 * @return The yaw value projected to [r_min, r_max].
 */
double projectYawToControlSpace(
    const cv::Mat& h, const PotentialTransform<ConstraintType::SOFT>& C_u,
    const double fx, const double px, const double yaw);

/**
 * @brief Compute the index of the throttle horizon is the greatest range.
 *
 * @pre Horizons shall be non-empty and of the same size. Damping index shall be
 * a valid horizon index.
 *
 * This function scans the throttle horizon and finds the one with the largest
 * defined control set. If the potential value for the found index does not
 * exceed the given inertia, the index is set to the damping index.
 *
 * @param throttle_h The horizon of throttle values.
 * @param potential_h The horizon of potential values.
 * @param inertia The inertia to overcome.
 * @param damp_idx The damping index.
 *
 * @return The throttle horizon index.
 */
int dampedMaxThrottleIndex(const cv::Mat& throttle_h,
                           const cv::Mat& potential_h, const double inertia,
                           const int damp_idx);
}  // namespace maeve_automation_core
