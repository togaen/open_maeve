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
#include "sequence_to_bag/sequence_to_bag.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/range/iterator_range.hpp>

namespace maeve_automation_core {

boost::optional<std::string> getPrefixedRow(const std::string& prefix,
                                            const std::string& str) {
  std::string row;
  std::istringstream is(str);
  while (std::getline(is, row)) {
    if (row.find(prefix) == 0) {
      auto suffix = row.substr(prefix.length());
      boost::trim(suffix);
      return suffix;
    }
  }
  return boost::none;
}

//------------------------------------------------------------------------------

std::vector<std::string> stringSplit(const std::string& str, const char delim) {
  std::string token;
  std::istringstream is(str);
  std::vector<std::string> tokens;
  while (std::getline(is, token, delim)) {
    tokens.push_back(std::move(token));
  }
  return tokens;
}

//------------------------------------------------------------------------------

sensor_msgs::CameraInfo synthesizeCameraInfoFromImageMsg(
    const sensor_msgs::ImagePtr& img_ptr) {
  return getUndistortedCameraInfo(img_ptr->header, img_ptr->width,
                                  img_ptr->height);
}

//------------------------------------------------------------------------------

sensor_msgs::CameraInfo getUndistortedCameraInfo(const std_msgs::Header& header,
                                                 const int image_width,
                                                 const int image_height) {
  sensor_msgs::CameraInfo cam_info_msg;

  cam_info_msg.header = header;
  cam_info_msg.width = image_width;
  cam_info_msg.height = image_height;
  cam_info_msg.distortion_model = "plumb_bob";

  cam_info_msg.D.resize(5);
  cam_info_msg.D[0] = 0.0;
  cam_info_msg.D[1] = 0.0;
  cam_info_msg.D[2] = 0.0;
  cam_info_msg.D[3] = 0.0;
  cam_info_msg.D[4] = 0.0;

  cam_info_msg.K = getIdentityMatrix<3>();
  cam_info_msg.K[2] = static_cast<double>(image_width) / 2.0;
  cam_info_msg.K[5] = static_cast<double>(image_height) / 2.0;

  cam_info_msg.R = getIdentityMatrix<3>();

  cam_info_msg.P[0] = 1.0;
  cam_info_msg.P[1] = 0.0;
  cam_info_msg.P[2] = static_cast<double>(image_width) / 2.0;
  cam_info_msg.P[3] = 0.0;

  cam_info_msg.P[4] = 1.0;
  cam_info_msg.P[5] = static_cast<double>(image_height) / 2.0;
  cam_info_msg.P[6] = 0.0;
  cam_info_msg.P[7] = 0.0;

  cam_info_msg.P[8] = 0.0;
  cam_info_msg.P[9] = 0.0;
  cam_info_msg.P[10] = 1.0;
  cam_info_msg.P[11] = 0.0;

  cam_info_msg.binning_x = 1;
  cam_info_msg.binning_y = 1;

  cam_info_msg.roi.width = image_width;
  cam_info_msg.roi.height = image_height;
  cam_info_msg.roi.x_offset = 0;
  cam_info_msg.roi.y_offset = 0;
  cam_info_msg.roi.do_rectify = false;

  return cam_info_msg;
}

//------------------------------------------------------------------------------

std::vector<boost::filesystem::path> getFileList(
    const std::string& path, boost::optional<std::string> extension) {
  std::vector<boost::filesystem::path> file_list;

  if (boost::filesystem::is_directory(path)) {
    for (auto& entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(path), {})) {
      const auto f = entry.path().filename().string();
      if (boost::filesystem::is_directory(entry) || f.empty() ||
          (f[0] == '.')) {
        continue;
      }
      if (extension && (entry.path().extension() != *extension)) {
        continue;
      }
      file_list.push_back(entry.path());
    }
  }

  return file_list;
}

}  // namespace maeve_automation_core
