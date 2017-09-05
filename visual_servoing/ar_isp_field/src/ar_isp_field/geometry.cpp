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
#include "maeve_automation_core/ar_isp_field/geometry.h"

namespace maeve_automation_core {
void arFillISP(const cv::Point2d& potential_value,
               const std::vector<cv::Point2d>& image_corner_points,
               cv::Mat& field) {
  std::vector<cv::Point2i> pts;
  pts.reserve(image_corner_points.size());
  std::for_each(std::begin(image_corner_points), std::end(image_corner_points),
                [&](const cv::Point2d& pt) {
                  pts.push_back(cv::Point2i(static_cast<int>(pt.x),
                                            static_cast<int>(pt.y)));
                });
  cv::fillConvexPoly(field, pts,
                     cv::Scalar(potential_value.x, potential_value.y));
}

namespace {
inline double squaredMag(const cv::Point2d& point) {
  return point.x * point.x + point.y * point.y;
}
}  // namespace

double arComputeMaxXY_Extent(const std::vector<cv::Point2d>& points) {
  static std::array<double, 6> extents;
  extents[0] = squaredMag(points[0] - points[1]);
  extents[1] = squaredMag(points[0] - points[2]);
  extents[2] = squaredMag(points[0] - points[3]);
  extents[3] = squaredMag(points[1] - points[2]);
  extents[4] = squaredMag(points[1] - points[3]);
  extents[5] = squaredMag(points[2] - points[3]);
  return std::sqrt(*std::max_element(std::begin(extents), std::end(extents)));
}

AR_Points arTagCornerPoints(const Eigen::Affine3d& camera_T_tag,
                            const AR_Points& model) {
  AR_Points points;

  // Transform each of ar_corner_points_ into points.
  for (auto i = 0; i < 4; ++i) {
    points[i] = camera_T_tag * model[i];
  }

  return points;
}

std::vector<cv::Point3d> arEigenPoints2OpenCV(const AR_Points& points) {
  std::vector<cv::Point3d> cv_points;

  cv_points.reserve(points.size());
  for (auto i = 0; i < 4; ++i) {
    cv_points.push_back(
        cv::Point3d(points[i].x(), points[i].y(), points[i].z()));
  }

  return cv_points;
}
}  // namespace maeve_automation_core
