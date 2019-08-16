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

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <array>
#include <vector>

namespace open_maeve {
/** @brief Set of points describing an AR tag. */
typedef std::array<Eigen::Vector3d, 4> AR_Points;

/**
 * @brief Draw a polygon gone defined by image_corner_points onto field and fill
 * with potentital_value.
 *
 * The polygon defined by image_corner_points must be convex and the points are
 * assumed to be given in CW order.
 *
 * @param potential_value The potential value to fill the polygon with.
 * @param image_corner_points The corner points of the polygon to draw.
 * @param field The field onto which to draw the polygon.
 */
void arFillISP(const cv::Point2d& potential_value,
               const std::vector<cv::Point2d>& image_corner_points,
               cv::Mat& field);

/**
 * @brief From a given AR tag pose, compute its corner points.
 *
 * This method assumes the pose is centered on the tag.
 *
 * @param Tx The AR tag pose.
 * @param model The set of points in the local frame of the AR tag.
 *
 * @return An array of corner points in CW order (assuming right hand system,
 * looking down at XY plane).
 */
AR_Points arTagCornerPoints(const Eigen::Affine3d& Tx, const AR_Points& model);

/**
 * @brief Compute the maximum extent in the XY projection of the given set of
 * AR tag points.
 *
 * @param points The AR tag corner points projected onto the image plane.
 *
 * @return The maximum extent.
 */
double arComputeMaxXY_Extent(const std::vector<cv::Point2d>& points);

/**
 * @brief Convert an array of Eigen::Vector3d points to cv::Point3d points.
 *
 * @param points The array of Eigen::Vector3d points.
 *
 * @return A vector of projected cv::Point3d objects.
 */
std::vector<cv::Point3d> arEigenPoints2OpenCV(const AR_Points& points);
}  // namespace open_maeve
