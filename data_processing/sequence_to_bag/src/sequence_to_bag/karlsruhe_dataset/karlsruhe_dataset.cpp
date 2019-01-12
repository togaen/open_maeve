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

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace maeve_automation_core {
namespace karlsruhe_dataset {

const std::string CV_IMAGE_ENCODING = sensor_msgs::image_encodings::BGR8;

std::string getCalibText(const std::string& data_set_path) {
  const auto calib_path = (data_set_path + "/" + CALIBRATION_FILENAME);
  return getFileText(calib_path);
}

//------------------------------------------------------------------------------

std::string getInsdataText(const std::string& data_set_path) {
  const auto insdata_path = (data_set_path + "/" + INSDATA_FILENAME);
  return getFileText(insdata_path);
}

//------------------------------------------------------------------------------

sensor_msgs::ImagePtr getImageMessage(const std_msgs::Header& header,
                                      const std::string& image_path) {
  cv::Mat img = cv::imread(image_path);
  return cv_bridge::CvImage(header, CV_IMAGE_ENCODING, img).toImageMsg();
}

//------------------------------------------------------------------------------

StereoImageFilePaths::StereoImageFilePaths(std::set<std::string> _left,
                                           std::set<std::string> _right)
    : left(std::move(_left)), right(std::move(_right)) {
  const bool empty = left.empty();
  const bool sizes_unequal = (left.size() != right.size());
  if (empty || sizes_unequal) {
    std::stringstream ss;
    ss << "Data set must contian an equal non-zero number of left and right "
          "images, but "
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

geometry_msgs::TransformStamped getStampedTransformFromOdomToCamera(
    const std_msgs::Header& header, const std::string& child_frame_id) {
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

  geometry_msgs::TransformStamped Tx_stamped;
  Tx_stamped.header = header;
  Tx_stamped.child_frame_id = child_frame_id;
  Tx_stamped.transform = Tx;

  return Tx_stamped;
}

}  // namespace karlsruhe_dataset
}  // namespace maeve_automation_core
