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
#include <ros/ros.h>

#include "maeve_automation_core/segmentation_taxonomy/segmentation_taxonomy.h"

int main(int argc, char* argv[]) {
  const auto node_name = std::string("segmentation_taxonomy");

  // Initialize ROS node.
  ros::init(argc, argv, node_name);

  maeve_automation_core::SegmentationTaxonomy t;
  if (!t.load("/home/jj56/data/sequences/s01/label_map.yaml", "s01")) {
    ROS_ERROR_STREAM("Failed to load parameters.");
    return EXIT_FAILURE;
  }
  ROS_INFO_STREAM("Loaded parameters.");

  // Kick it off.
  ros::spin();

  // Done.
  return EXIT_SUCCESS;
}
