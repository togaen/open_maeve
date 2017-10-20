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

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>

#include <algorithm>
#include <limits>
#include <string>

#include "maeve_automation_core/isp_controller_2d/ros_interface.h"
#include "maeve_automation_core/isp_field/isp_field.h"
#include "maeve_automation_core/isp_field/visualize.h"
#include "maeve_automation_core/segmentation_taxonomy/types.h"
#include "segmentation_isp_field/lib.h"

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
  segmentation_sub_ = it_.subscribeCamera(
      params_.segmentation_sequence_topic, 10,
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

  // Instantiate transforms.
  sc_ = PotentialTransform<ConstraintType::SOFT>(
      params_.soft_constraint_transform);

  // Load any guidance weights that live on the parameter server.
  loadGuidancePotentials(nh_, taxonomy_, sc_, guidance_potentials_);

  // Set up command handler.
  command2d_mgr_.initialize(nh_, params_.control_command_input_topic);

  // Control command output publisher.
  control_command_output_pub_ =
      nh_.advertise<controller_interface_msgs::Command2D>(
          params_.control_command_output_topic, 1);

  // Visualize?
  viz_isp_field_pub_ = it_.advertise(params_.viz_isp_field_topic, 1);
  std::for_each(
      std::begin(params_.visualize_horizons),
      std::end(params_.visualize_horizons), [&](const std::string& str) {
        viz_horizon_pubs_[str] = it_.advertise("viz_" + str + "_horizon", 1);
      });
}

void SegmentationFieldNodeHandler::loadGuidancePotentials(
    const ros::NodeHandle& nh, const SegmentationTaxonomy& taxonomy,
    const PotentialTransform<ConstraintType::SOFT>& sc,
    GuidancePotentials& guidance_potentials) {
  std::for_each(std::begin(taxonomy.classes), std::end(taxonomy.classes),
                [&](const LabelClasses::value_type& p) {
                  const auto& class_name = p.first;
                  auto weight = NaN;
                  if (nh.getParam("guidance_weights/" + class_name, weight)) {
                    guidance_potentials[class_name] =
                        sc(cv::Point2d(weight, 0.0));
                  } else {
                    ROS_WARN_STREAM("Guidance weight for class '"
                                    << class_name << "' not found.");
                  }
                });
}

void SegmentationFieldNodeHandler::segmentationSequenceCallback(
    const sensor_msgs::ImageConstPtr& msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // Make sure field maps have storage allocated, but do it only once.
  static auto init = true;
  if (init) {
    // Initialize camera model.
    camera_model_.fromCameraInfo(info_msg);

    // Initialize ISP controller.
    params_.isp_controller_params.principal_point_x = camera_model_.cx();
    params_.isp_controller_params.focal_length_x = camera_model_.fx();
    isp_controller_ = ISP_Controller2D(params_.isp_controller_params);
    init = false;
  }

  // Convert ROS image to OpenCV.
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }

  // Construct ISP field.
  auto guidance_field = zeroISP_Field(msg->width, msg->height);

  // Run through label map, construct guidance field.
  std::for_each(std::begin(guidance_potentials_),
                std::end(guidance_potentials_),
                [&](const GuidancePotentials::value_type& p) {
                  guidance_field =
                      guidance_field +
                      extractGuidanceField(
                          cv_ptr->image, taxonomy_.classes[p.first], p.second);
                });

  // Get desired control.
  ControlCommand u_d;
  if (const auto cmd_msg = command2d_mgr_.mostRecentMsg()) {
    u_d = command2D_Msg2ControlCommand(*cmd_msg);
    if (!u_d.valid()) {
      u_d = params_.default_guidance_control;
      ROS_ERROR_STREAM("u_d not valid: "
                       << u_d << ", sending default guidance control: " << u_d);
    }
  } else {
    u_d = params_.default_guidance_control;
  }

  // Compute SD control.
  if (!isp_controller_.isInitialized()) {
    ROS_ERROR_STREAM("ISP controller not initialized.");
    return;
  }
  const auto u_star = isp_controller_.SD_Control(guidance_field, u_d);
  // ROS_INFO_STREAM("u_d: " << u_d << ", u_star: " << u_star);

  // Publish control.
  control_command_output_pub_.publish(
      controlCommand2Command2D_Msg(u_star, msg->header));

  // Visualize ISP field.
  const auto viz_field = computeISPFieldVisualization(
      guidance_field, params_.viz_potential_bounds[0],
      params_.viz_potential_bounds[1]);
  sensor_msgs::ImagePtr viz_field_msg =
      cv_bridge::CvImage(msg->header, "bgr8", viz_field).toImageMsg();
  viz_isp_field_pub_.publish(viz_field_msg);

  // Horizon visualizations.
  for (const auto& p : viz_horizon_pubs_) {
    const auto ht = ISP_Controller2D::stringToHorizonType(p.first);
    auto lower_bound = sc_.shapeParameters().range_min;
    auto upper_bound = sc_.shapeParameters().range_max;
    if ((ht == ISP_Controller2D::HorizonType::YAW_GUIDANCE) ||
        (ht == ISP_Controller2D::HorizonType::GUIDANCE) ||
        (ht == ISP_Controller2D::HorizonType::GUIDED_THROTTLE) ||
        (ht == ISP_Controller2D::HorizonType::CONTROL_SET_GUIDANCE)) {
      lower_bound = 0.0;
      upper_bound = 1.0;
    }
    visualizeHorizon(msg->header, msg->height, ht, p.second, lower_bound,
                     upper_bound);
  }
}

void SegmentationFieldNodeHandler::visualizeHorizon(
    const std_msgs::Header& header, const int height,
    const ISP_Controller2D::HorizonType ht,
    const image_transport::Publisher& publisher, const double lower_bound,
    const double upper_bound) const {
  const auto& horizon = isp_controller_.inspectHorizon(ht);
  const auto viz_horizon = computeHorizonVisualization(
      horizon, params_.horizon_viz_height, params_.horizon_viz_height,
      lower_bound, upper_bound);
  sensor_msgs::ImagePtr viz_horizon_msg =
      cv_bridge::CvImage(header, "mono8", viz_horizon).toImageMsg();
  publisher.publish(viz_horizon_msg);
}
}  // namespace maeve_automation_core