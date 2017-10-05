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
#include "sequence_to_bag/io.h"

#include <perception_interface_msgs/ImagePair.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosgraph_msgs/Clock.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdlib>
#include <string>

namespace {
const auto CLOCK_HZ_DEFAULT = 1000;
double getClockHz(const std::string& str) {
  char* pEnd = nullptr;
  const auto hz = std::strtod(str.c_str(), &pEnd);
  if (str.empty() || (*pEnd != '\0')) {
    return CLOCK_HZ_DEFAULT;
  }
  return hz;
}
}  // namespace

int main(int argc, char** argv) {
  // Make sure all required arguments are specified.
  if (argc < 4) {
    ROS_FATAL_STREAM(
        "Provided " << (argc - 1)
                    << " arguments. Must provide at least three arguments: "
                       "'data-set-path' 'bag-output-path' 'image-pair-topic'.");
    return EXIT_FAILURE;
  }
  const auto data_set_path = std::string(argv[1]);
  const auto output_path = std::string(argv[2]);
  const auto image_pair_topic = std::string(argv[3]);
  const auto clock_hz_param =
      getClockHz((argc > 4) ? std::string(argv[4]) : std::string(""));

  // Let people know what's going on.
  ROS_INFO_STREAM("\nUsing data set path: "
                  << data_set_path << "\nUsing output path: " << output_path
                  << "\nUsing image pair topic: " << image_pair_topic
                  << "\nClock hz: " << clock_hz_param);

  // Read meta info from data set.
  YAML::Node config;
  const auto meta_path = data_set_path + "/meta.yaml";
  try {
    config = YAML::LoadFile(meta_path);
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("Exception caught while loading file '"
                     << meta_path
                     << "'; does the file exist? Exception message: "
                     << e.what());
    return EXIT_FAILURE;
  }

  // \TODO(me) Maybe check that the keys exist.

  // Get meta info.
  const auto raw_image_dir =
      data_set_path + "/" + config["sequence_meta"]["raw"].as<std::string>();
  const auto seg_image_dir =
      data_set_path + "/" +
      config["sequence_meta"]["segmented"].as<std::string>();
  const auto fps = config["sequence_meta"]["fps"].as<double>();
  const auto label_map_params = config["sequence_meta"]["label_map"];
  if (!label_map_params.IsMap()) {
    ROS_FATAL_STREAM("Malformed meta.yaml; expected label_map.");
    return EXIT_FAILURE;
  }

  // Get files.
  const auto raw_files = maeve_automation_core::getFileList(raw_image_dir);
  const auto raw_files_idx =
      maeve_automation_core::getSortedIndexedFileList(raw_files);
  const auto raw_images_idx =
      maeve_automation_core::getSortedIndexedImages(raw_files_idx);

  const auto seg_files = maeve_automation_core::getFileList(seg_image_dir);
  const auto seg_files_idx =
      maeve_automation_core::getSortedIndexedFileList(seg_files);
  const auto seg_images_idx =
      maeve_automation_core::getSortedIndexedImages(seg_files_idx);

  if (raw_images_idx.size() != seg_files_idx.size()) {
    ROS_FATAL_STREAM("Mismatch in data set. Images count: "
                     << raw_images_idx.size()
                     << " should equal Segmented count: "
                     << seg_images_idx.size());
    return EXIT_FAILURE;
  }

  // Build rosbag.
  rosbag::Bag bag;
  bag.open(output_path + "/" + config["sequence_meta"]["name"].as<std::string>() + ".bag",
           rosbag::bagmode::Write);
  ros::Time::init();
  auto t = ros::Time::now();
  std::for_each(std::begin(raw_images_idx), std::end(raw_images_idx),
                [&](const std::map<int, sensor_msgs::ImagePtr>::value_type& p) {
                  if (seg_images_idx.find(p.first) == seg_images_idx.end()) {
                    ROS_ERROR_STREAM("Failed to find image index "
                                     << p.first << " in segmented data set.");
                    return;
                  }

                  const auto msg1 = p.second;
                  const auto msg2 = seg_images_idx.at(p.first);

                  const auto timestamp = t + ros::Duration(p.first);

                  msg1->header.stamp = timestamp;
                  msg2->header.stamp = timestamp;

                  bag.write("image1", timestamp, *msg1);
                  bag.write("image2", timestamp, *msg2);

                });

  ROS_INFO_STREAM("\nLoaded meta file: "
                  << meta_path << "\nRaw image path: " << raw_image_dir
                  << "\nSegmented image path: " << seg_image_dir
                  << "\nfps: " << fps);

  // Generate bag file.
  bag.close();
  return EXIT_SUCCESS;
}
