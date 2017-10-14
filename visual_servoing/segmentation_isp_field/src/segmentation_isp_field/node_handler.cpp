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

#include "segmentation_isp_field/node_handler.h"

#include <algorithm>
#include <limits>

#include "maeve_automation_core/isp_controller_2d/ros_interface.h"
#include "maeve_automation_core/isp_field/isp_field.h"
#include "maeve_automation_core/segmentation_taxonomy/types.h"

namespace maeve_automation_core {
namespace {
const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

SegmentationFieldNodeHandler::SegmentationFieldNodeHandler(
    const std::string& node_name)
    : nh_(node_name), it_(nh_) {
  // Load node parameters.
  if (!params_.load(nh_)) {
    ROS_FATAL_STREAM("Failed to load parameters. Fatal error.");
    return;
  }

  // Register callback.
  segmentation_sub_ = it_.subscribe(
      params_.segmentation_sequence_topic, 1,
      &SegmentationFieldNodeHandler::segmentationSequenceCallback, this);

  // Set up command handler.
  command2d_mgr_.initialize(nh_, params_.control_command_input_topic);

  // Control command output publisher.
  control_command_output_pub_ =
      nh_.advertise<controller_interface_msgs::Command2D>(
          params_.control_command_output_topic, 1);

  // Load the data set taxonomy.
  if (!taxonomy_.load(params_.label_map_path, params_.data_set_name)) {
    ROS_FATAL_STREAM("Failed to load data set taxonomy. Fatal error.");
    return;
  }

  ROS_INFO_STREAM("Loaded Taxonomy:\n" << taxonomy_);

  // Load any guidance weights that live on the parameter server.
  loadGuidanceWeights(nh_, taxonomy_, guidance_weights_);

  // Instantiate transforms.
  hc_ = PotentialTransform<ConstraintType::HARD>(
      params_.hard_constraint_transform);
  sc_ = PotentialTransform<ConstraintType::SOFT>(
      params_.soft_constraint_transform);

  // Visualize?
  if (!params_.viz_isp_field_topic.empty()) {
    viz_isp_field_pub_ = it_.advertise(params_.viz_isp_field_topic, 1);
  }
}

void SegmentationFieldNodeHandler::loadGuidanceWeights(
    const ros::NodeHandle& nh, const SegmentationTaxonomy& taxonomy,
    std::unordered_map<std::string, double>& guidance_weights) {
  std::for_each(std::begin(taxonomy.classes), std::end(taxonomy.classes),
                [&](const LabelClasses::value_type& p) {
                  const auto& class_name = p.first;
                  auto weight = NaN;
                  if (nh.getParam("guidance_weights/" + class_name, weight)) {
                    guidance_weights[class_name] = weight;
                  } else {
                    ROS_WARN_STREAM("Guidance weight for class '"
                                    << class_name << "' not found.");
                  }
                });
}

void SegmentationFieldNodeHandler::segmentationSequenceCallback(
    const sensor_msgs::ImageConstPtr& msg) {
  // Construct ISP field.
  auto guidance_field = zeroISP_Field(msg->width, msg->height);

  // Run through label map, construct guidance field.

  // Feed guidance field to controller.

  // Compute control.

  // Publish control.
}
}  // namespace maeve_automation_core
