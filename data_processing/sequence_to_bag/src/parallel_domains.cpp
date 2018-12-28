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

#include "sequence_to_bag/parallel_domains/parallel_domains.h"

#include <algorithm>
#include <map>
#include <string>

#include <camera_calibration_parsers/parse.h>

namespace po = boost::program_options;

int main(int argc, char** argv) {
  boost::optional<std::string> data_set_path_opt;
  boost::optional<std::string> output_path_opt;
  boost::optional<std::string> image_1_topic_opt;
  boost::optional<std::string> image_2_topic_opt;

  po::options_description desc(
      "Available arguments. All required arguments must be set:");
  desc.add_options()("help", "Print help and exit.")(
      "data-set-path", po::value(&data_set_path_opt),
      "[Required] Absolute path to the data set.")(
      "bag-output-dir", po::value(&output_path_opt),
      "[Required]: Absolute path to the directory that will contain the output "
      "bag file.")("raw-image-camera-name", po::value(&image_1_topic_opt),
                   "[Required]: Camera name to use for the raw image stream.")(
      "segmented-image-camera-name", po::value(&image_2_topic_opt),
      "[Required]: Camera name to use for the segmented image stream.");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  const auto help_requested = vm.count("help");
  const auto required_arg_not_set = !(data_set_path_opt && output_path_opt &&
                                      image_1_topic_opt && image_2_topic_opt);
  if (help_requested || required_arg_not_set) {
    std::cout << desc << "\n";
    return EXIT_SUCCESS;
  }

  const auto data_set_path = *data_set_path_opt;
  const auto output_path = *output_path_opt;
  const auto image_1_topic = *image_1_topic_opt;
  const auto image_2_topic = *image_2_topic_opt;
  const auto clock_hz_param =
      maeve_automation_core::parallel_domains::getClockHz(
          (argc > 5) ? std::string(argv[5]) : std::string(""));

  // Let people know what's going on.
  ROS_INFO_STREAM("\nUsing data set path: "
                  << data_set_path << "\nUsing output path: " << output_path
                  << "\nUsing image 1 topic: " << image_1_topic
                  << "\nUsing image 2 topic: " << image_2_topic
                  << "\nClock hz: " << clock_hz_param);

  // Get meta information.
  const auto meta_info =
      maeve_automation_core::parallel_domains::getMetaInfo(data_set_path);
  if (!meta_info) {
    ROS_FATAL_STREAM(
        "Failed retrieving meta info; does "
        << maeve_automation_core::parallel_domains::constructMetaYamlPath(
               data_set_path)
        << " exist?");
    return EXIT_FAILURE;
  }
  ROS_INFO_STREAM("Meta info:\n" << *meta_info);

  // Get files.
  std::map<int, sensor_msgs::ImagePtr> raw_images_idx;
  std::map<int, sensor_msgs::ImagePtr> seg_images_idx;
  if (auto m =
          maeve_automation_core::parallel_domains::getSortedIndexedImageLists(
              meta_info->raw_image_dir, meta_info->seg_image_dir)) {
    std::tie(raw_images_idx, seg_images_idx) = *m;
  } else {
    ROS_FATAL_STREAM("Error in data set image sets.");
    return EXIT_FAILURE;
  }

  if (seg_images_idx.empty() || raw_images_idx.empty()) {
    ROS_FATAL_STREAM("No files found.");
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
        maeve_automation_core::synthesizeCameraInfoFromImageMsg(seg_image_ptr);
  }

  // Time between frames.
  const auto frame_duration = 1.0 / meta_info->fps;

  // Build rosbag.
  rosbag::Bag bag;
  ros::Time::init();
  auto t = ros::Time::now();
  bag.open(output_path + "/" + meta_info->name + ".bag",
           rosbag::bagmode::Write);
  auto elapsed_time = 0.0;
  std::for_each(
      std::begin(raw_images_idx), std::end(raw_images_idx),
      [&](const std::map<int, sensor_msgs::ImagePtr>::value_type& p) {
        if (seg_images_idx.find(p.first) == seg_images_idx.end()) {
          ROS_ERROR_STREAM("Failed to find image index "
                           << p.first << " in segmented data set.");
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
