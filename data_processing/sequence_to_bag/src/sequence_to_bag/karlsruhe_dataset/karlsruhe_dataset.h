/*
 * Copyright 2018 Maeve Automation
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

#include <geometry_msgs/Transform.h>

#include <set>
#include <string>

#include "sequence_to_bag/karlsruhe_dataset/calib.h"
#include "sequence_to_bag/sequence_to_bag.h"

namespace maeve_automation_core {
namespace karlsruhe_dataset {
static constexpr auto CALIBRATION_FILENAME = "calib.txt";
static constexpr auto IMAGE_EXTENSION = ".png";
static constexpr auto LEFT_IMAGE_PREFIX = "I1";

/**
 * @brief Container for sets of paths of left/right image pairs
 *
 * @post The left/right sets are of equal size if construction succeeds
 *
 * @note An exception is thrown if the left/right sets are unequal in size
 */
struct StereoImageFilePaths {
  const std::set<std::string> left;
  const std::set<std::string> right;
  StereoImageFilePaths(std::set<std::string> _left,
                       std::set<std::string> _right);
};  // struct StereoImageFilePaths

/**
 * @brief Get the set of left/right image pairs for a given dataset path
 *
 * @note An exception is thrown if left and right image counts differ
 */
StereoImageFilePaths getStereoImageFiles(const std::string& dataset_path);

/**
 * @brief Test whether a filename belongs to left camera
 */
bool isLeftImage(const std::string& filename);

/**
 * @brief Load the calibration file from the given data set path
 *
 * @note An exception is thrown if the path is malformed
 */
calib::stereoCameraInfo getCameraInfo(const ros::Time& camera_info_timestamp,
                                      const std::string& camera_name,
                                      const int image_width,
                                      const int image_height,
                                      const std::string& dataset_path);

/**
 * @brief Transform a point from odom frame to camera frame
 */
geometry_msgs::Transform getTransformFromOdomToCamera();

}  // namespace karlsruhe_dataset
}  // namespace maeve_automation_core
