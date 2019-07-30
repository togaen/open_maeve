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

#include "sequence_to_bag/parallel_domain/parallel_domain.h"

#include <algorithm>
#include <map>
#include <string>

#include <camera_calibration_parsers/parse.h>

namespace {
namespace po = boost::program_options;
}  // namespace

int main(int argc, char** argv) {
  // Command line arguments
  boost::optional<std::string> data_set_path_opt;
  boost::optional<std::string> output_path_opt;
  constexpr auto RAW_IMAGE_CAMERA_DEFAULT = "raw";
  constexpr auto SEGMENTED_IMAGE_CAMERA_DEFAULT = "segmented";

  po::options_description desc(
      "Parallel Domain data set sequencer. See README.md for "
      "details.\nAvailable options are listed below. Arguments without "
      "default values are required",
      maeve_core::PROGRAM_OPTIONS_LINE_LENGTH);
  desc.add_options()("help,h", "Print help and exit.")(
      "data-set-path,d", po::value(&data_set_path_opt)->required(),
      "Absolute path to the data set.")(
      "bag-output-dir,o", po::value(&output_path_opt)->required(),
      "Absolute path to the directory for the output bag file.")(
      "raw-image-camera,r",
      po::value<std::string>()->default_value(RAW_IMAGE_CAMERA_DEFAULT),
      "Camera name to use for the raw image stream.")(
      "segmented-image-camera,s",
      po::value<std::string>()->default_value(SEGMENTED_IMAGE_CAMERA_DEFAULT),
      "Camera name to use for the segmented image stream.");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  const auto help_requested = vm.count("help");
  if (help_requested) {
    std::cout << desc << "\n";
    return EXIT_SUCCESS;
  }

  try {
    po::notify(vm);
  } catch (boost::program_options::required_option& e) {
    std::cerr << "Ensure that all required options are specified: " << e.what()
              << "\n\n";
    std::cerr << desc << "\n";
    return EXIT_FAILURE;
  }

  const auto data_set_path = *data_set_path_opt;
  const auto output_path = *output_path_opt;
  const auto image_1_topic = vm["raw-image-camera"].as<std::string>();
  const auto image_2_topic = vm["segmented-image-camera"].as<std::string>();

  // Get meta information.
  const auto meta_info =
      maeve_core::parallel_domain::getMetaInfo(data_set_path);
  if (!meta_info) {
    std::cerr << "Failed retrieving meta info; does "
              << maeve_core::parallel_domain::constructMetaYamlPath(
                     data_set_path)
              << " exist?\n";
    return EXIT_FAILURE;
  }
  std::cout << "Meta info:\n" << *meta_info << "\n";

  // Get files.
  std::map<int, sensor_msgs::ImagePtr> raw_images_idx;
  std::map<int, sensor_msgs::ImagePtr> seg_images_idx;
  if (auto m =
          maeve_core::parallel_domain::getSortedIndexedImageLists(
              meta_info->raw_image_dir, meta_info->seg_image_dir)) {
    std::tie(raw_images_idx, seg_images_idx) = *m;
  } else {
    std::cerr << "Error in data set image sets.\n";
    return EXIT_FAILURE;
  }

  if (seg_images_idx.empty() || raw_images_idx.empty()) {
    std::cerr << "No files found.\n";
    return EXIT_FAILURE;
  }

  // Get camera calibration.
  std::string camera_name;
  sensor_msgs::CameraInfo cam_info_msg;
  const auto success = camera_calibration_parsers::readCalibration(
      data_set_path + "/camera_info.yaml", camera_name, cam_info_msg);
  if (!success) {
    const auto& seg_image_ptr = seg_images_idx.begin()->second;
    camera_name = "camera";
    cam_info_msg =
        maeve_core::synthesizeCameraInfoFromImageMsg(seg_image_ptr);
  }

  // Time between frames.
  const auto frame_duration = 1.0 / meta_info->fps;

  // Build rosbag.
  rosbag::Bag bag;
  ros::Time::init();
  auto t = ros::Time::now();
  bag.open(output_path + "/" + meta_info->name +
               maeve_core::BAG_FILE_EXTENSION,
           rosbag::bagmode::Write);
  auto elapsed_time = 0.0;
  std::for_each(
      std::begin(raw_images_idx), std::end(raw_images_idx),
      [&](const std::map<int, sensor_msgs::ImagePtr>::value_type& p) {
        if (seg_images_idx.find(p.first) == seg_images_idx.end()) {
          std::cerr << "Failed to find image index " << p.first
                    << " in segmented data set.\n";
          return;
        }

        const auto msg1 = p.second;
        const auto msg2 = seg_images_idx.at(p.first);

        const auto timestamp = t + ros::Duration(elapsed_time);

        msg1->header.stamp = timestamp;
        msg2->header.stamp = timestamp;
        cam_info_msg.header.stamp = timestamp;

        bag.write(image_1_topic + "/image", timestamp, *msg1);
        bag.write(image_1_topic + "/camera_info", timestamp, cam_info_msg);
        bag.write(image_2_topic + "/image", timestamp, *msg2);
        bag.write(image_2_topic + "/camera_info", timestamp, cam_info_msg);

        elapsed_time += frame_duration;
      });

  // Generate bag file.
  bag.close();
  return EXIT_SUCCESS;
}
