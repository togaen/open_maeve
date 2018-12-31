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
#include "sequence_to_bag/karlsruhe_dataset/karlsruhe_dataset.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace maeve_automation_core {
namespace karlsruhe_dataset {

StereoImageFilePaths::StereoImageFilePaths(std::set<std::string> _left,
                                           std::set<std::string> _right)
    : left(std::move(_left)), right(std::move(_right)) {
  if (left.size() != right.size()) {
    std::stringstream ss;
    ss << "Data set must contian an equal number of left and right images, but "
       << left.size() << " and " << right.size()
       << " were found, respectively.";
    throw std::invalid_argument(ss.str());
  }
}

//------------------------------------------------------------------------------

StereoImageFilePaths getStereoImageFiles(const std::string& dataset_path) {
  const auto file_list =
      getFileList(dataset_path, std::string(IMAGE_EXTENSION));

  std::set<std::string> left;
  std::set<std::string> right;
  std::for_each(std::begin(file_list), std::end(file_list),
                [&left, &right](const boost::filesystem::path& path) {
                  if (isLeftImage(path.filename().string())) {
                    left.insert(path.string());
                  } else {
                    right.insert(path.string());
                  }
                });

  return StereoImageFilePaths(std::move(left), std::move(right));
}

//------------------------------------------------------------------------------

bool isLeftImage(const std::string& filename) {
  return (filename.find(LEFT_IMAGE_PREFIX) == 0);
}

//------------------------------------------------------------------------------

calib::stereoCameraInfo getCameraInfo(const ros::Time& camera_info_timestamp,
                                      const std::string& camera_name,
                                      const int image_width,
                                      const int image_height,
                                      const std::string& dataset_path) {
  const auto calib_path = (dataset_path + "/" + CALIBRATION_FILENAME);
  std::ifstream ifs(calib_path);
  std::string text((std::istreambuf_iterator<char>(ifs)),
                   (std::istreambuf_iterator<char>()));

  return calib::convertToCameraInfo(camera_info_timestamp, camera_name, text,
                                    image_width, image_height);
}

//------------------------------------------------------------------------------

geometry_msgs::Transform getTransformFromOdomToCamera() {
  geometry_msgs::Vector3 T;
  T.x = 1.6;
  T.y = 0.05;
  T.z = 0.6;
  geometry_msgs::Quaternion R;
  R.w = 0.999999756306074;
  R.x = 0.0;
  R.y = -0.000698131644088;
  R.z = 0.0;
  geometry_msgs::Transform Tx;
  Tx.translation = T;
  Tx.rotation = R;
  return Tx;
}

}  // namespace karlsruhe_dataset
}  // namespace maeve_automation_core
