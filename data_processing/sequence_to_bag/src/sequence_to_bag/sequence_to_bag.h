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
 * @brief Get a list of all non-hidden files in the given path.
 *
 * @param path The path of the directory to list files for.
 *
 * @return A vector of fully qualifed filenames. If 'path' is not a valid
 * directory the returned vector will be empty.
 */
std::vector<boost::filesystem::path> getFileList(const std::string& path);

}  // namespace maeve_automation_core
