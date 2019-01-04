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

#include <algorithm>
#include <cstdlib>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>

#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace maeve_automation_core {

/** @brief When printing program options to terminal, use this line length */
static constexpr unsigned int PROGRAM_OPTIONS_LINE_LENGTH = 120u;

static constexpr auto BAG_FILE_EXTENSION = ".bag";
static constexpr auto TF_TOPIC = "/tf";
static constexpr auto STATIC_TF_TOPIC = "/tf_static";

/**
 * @brief Get an identity matrix with side length 'size'
 *
 * This function is only usefule for initializing matrices stored in ROS
 * messages. Otherwise, use std::array.
 *
 * @note The returned matrix is flat
 */
template <int SIZE>
boost::array<double, (SIZE * SIZE)> getIdentityMatrix() {
  boost::array<double, (SIZE * SIZE)> matrix;
  std::for_each(std::begin(matrix), std::end(matrix),
                [](double& val) { val = 0.0; });

  for (auto i = 0; i < SIZE; ++i) {
    const auto idx = (i + (i * SIZE));
    matrix[idx] = 1.0;
  }
  return matrix;
}

/**
 * @brief Get the contents of the file at 'path' and load it into a string
 */
std::string getFileText(const std::string& path);

/**
 * @brief Get the first row, if any, from 'str' prefixed by 'prefix'
 *
 * @note Leading and trailing whitespace is trimmed from the result
 *
 * For input:
 *  prefix = "B:"
 *  str    =
 *     "A: This is row 1
 *      B: This is row 2
 *      C: This is row 3
 *      B: This is row 4"
 *
 * The output is: "This is row 2"
 */
boost::optional<std::string> getPrefixedRow(const std::string& prefix,
                                            const std::string& str);

/**
 * @brief Tokenize a string according to the given delimiter
 */
std::vector<std::string> stringSplit(const std::string& str, const char delim);

/**
 * @brief Synthesize a camera info message from an image message.
 *
 * This message assumes unit focal lengths, no distortion, and perfect optical
 * center.
 *
 * @param img_ptr Image message pointer.
 *
 * @return A camera info message.
 */
sensor_msgs::CameraInfo synthesizeCameraInfoFromImageMsg(
    const sensor_msgs::ImagePtr& img_ptr);

/**
 * @brief Get camera info for a perfect, undistored camera
 *
 * @note A "plumb bob" distortion model is set
 */
sensor_msgs::CameraInfo getUndistortedCameraInfo(const std_msgs::Header& header,
                                                 const int image_width,
                                                 const int image_height);

/**
 * @brief Get a list of all non-hidden files in the given path.
 *
 * @param path       The path of the directory to list files for.
 * @param extension  Optional: only include files with the given extension
 *
 * @note When specifying an extension, include the dot (e.g., ".png")
 *
 * @return A vector of fully qualifed filenames. If 'path' is not a valid
 * directory the returned vector will be empty.
 */
std::vector<boost::filesystem::path> getFileList(
    const std::string& path,
    boost::optional<std::string> extension = boost::none);

}  // namespace maeve_automation_core
