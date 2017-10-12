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

#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosgraph_msgs/Clock.h>

#include <algorithm>
#include <cstdlib>
#include <string>

int main(int argc, char** argv) {
  // Make sure all required arguments are specified.
  if (argc < 4) {
    ROS_FATAL_STREAM(
        "Provided "
        << (argc - 1)
        << " arguments. Must provide at least four arguments: "
           "'data-set-path' 'bag-output-dir' 'camera-image-topic-name' "
           "'segmented-image-topic-name'");
    return EXIT_FAILURE;
  }
  const auto data_set_path = std::string(argv[1]);
  const auto output_path = std::string(argv[2]);
  const auto image_1_topic = std::string(argv[3]);
  const auto image_2_topic = std::string(argv[4]);
  const auto clock_hz_param = maeve_automation_core::getClockHz(
      (argc > 5) ? std::string(argv[5]) : std::string(""));

  // Let people know what's going on.
  ROS_INFO_STREAM("\nUsing data set path: "
                  << data_set_path << "\nUsing output path: " << output_path
                  << "\nUsing image 1 topic: " << image_1_topic
                  << "\nUsing image 2 topic: " << image_2_topic
                  << "\nClock hz: " << clock_hz_param);

  // Get meta information.
  const auto meta_info = maeve_automation_core::getMetaInfo(data_set_path);
  if (!meta_info) {
    ROS_FATAL_STREAM(
        "Failed retrieving meta info; does "
        << maeve_automation_core::constructMetaYamlPath(data_set_path)
        << " exit?");
    return EXIT_FAILURE;
  }
  ROS_INFO_STREAM("Meta info:\n" << *meta_info);

  // Get files.
  std::map<int, sensor_msgs::ImagePtr> raw_images_idx;
  std::map<int, sensor_msgs::ImagePtr> seg_images_idx;
  if (auto m = maeve_automation_core::getSortedIndexedImageLists(
          meta_info->raw_image_dir, meta_info->seg_image_dir)) {
    std::tie(raw_images_idx, seg_images_idx) = *m;
  } else {
    ROS_FATAL_STREAM("Error in data set image sets.");
    return EXIT_FAILURE;
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
  std::for_each(std::begin(raw_images_idx), std::end(raw_images_idx),
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

                  bag.write(image_1_topic, timestamp, *msg1);
                  bag.write(image_2_topic, timestamp, *msg2);

                  elapsed_time += frame_duration;
                });

  // Generate bag file.
  bag.close();
  return EXIT_SUCCESS;
}
