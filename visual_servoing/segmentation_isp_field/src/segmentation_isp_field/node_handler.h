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
#pragma once

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

#include <string>
#include <unordered_map>

#include "maeve_automation_core/controller_interface_msgs/command2d_manager.h"
#include "maeve_automation_core/isp_controller_2d/isp_controller_2d.h"
#include "maeve_automation_core/isp_field/potential_transforms.h"
#include "maeve_automation_core/segmentation_taxonomy/segmentation_taxonomy.h"
#include "segmentation_isp_field/params.h"

namespace maeve_automation_core {
/**
 * @brief Interface between ROS and the segmentation-based ISP controller.
 */
class SegmentationFieldNodeHandler {
 public:
  /**
   * @brief Construct handler instance and register callbacks/subscribers.
   *
   * @param node_name The node name used to construct the ROS node handle.
   */
  explicit SegmentationFieldNodeHandler(const std::string& node_name);

 private:
  /** @brief Map of class name to guidance potential value. */
  typedef std::unordered_map<std::string, cv::Point2d> GuidancePotentials;

  /**
   * @brief Helper function for visualizing horizon structures.
   *
   * @param header Message header for published horizon visualization.
   * @param height Height of the visualization image.
   * @param cs Which of the horizon structures to visualize.
   * @param publisher The publisher to use to publish the visualization.
   */
  void visualizeHorizon(const std_msgs::Header& header, const int height,
                        const ISP_Controller2D::ControlStructure cs,
                        const image_transport::Publisher& publisher) const;

  /**
   * @brief Callback that fires when a new image segmentation is received.
   *
   * @param msg The segmentation image message.
   */
  void segmentationSequenceCallback(const sensor_msgs::ImageConstPtr& msg);

  /**
   * @brief For a given taxonomy, load any specified guidance weights from the
   * parameter server.
   *
   * @note Guidance weights must live in a node-relative namespace called
   * 'guidance_weights'.
   *
   * @param nh The ROS node handle used for interacting with the parameter
   * server.
   * @param taxonomy The taxonomy to load guidance weights for.
   * @param sc The soft constraint transform to apply to the guidance weights.
   * @param guidance_potentials The set of guidance weights to fill.
   */
  static void loadGuidancePotentials(
      const ros::NodeHandle& nh, const SegmentationTaxonomy& taxonomy,
      const PotentialTransform<ConstraintType::SOFT>& sc,
      GuidancePotentials& guidance_potentials);

  /** @brief Manager for retrieving most recent control commands. */
  Command2D_Manager command2d_mgr_;

  /** @brief Compute control commands from ISP field. */
  ISP_Controller2D isp_controller_;

  /** @brief Set of guidance pontentials. */
  GuidancePotentials guidance_potentials_;

  /** @brief Parser for loading data set taxonomy. */
  SegmentationTaxonomy taxonomy_;
  /** @brief Camera image subscriber. */
  image_transport::Subscriber segmentation_sub_;
  /** @brief ISP field visualization publisher. */
  image_transport::Publisher viz_isp_field_pub_;
  /** @brief Horizon visualization publishers. */
  std::unordered_map<std::string, image_transport::Publisher> viz_horizon_pubs_;
  /** @brief The ROS node handle. */
  ros::NodeHandle nh_;
  /** @brief The ROS image transport object. */
  image_transport::ImageTransport it_;
  /** @brief Desired control command subscriber. */
  ros::Subscriber control_command_input_sub_;
  /** @brief Control command publisher. */
  ros::Publisher control_command_output_pub_;
  /** @brief Node params. */
  SegmentationFieldParams params_;
  /** @brief The soft constraint transform. */
  PotentialTransform<ConstraintType::SOFT> sc_;
};  // class SegmentationFieldNodeHandler
}  // namespace maeve_automation_core
