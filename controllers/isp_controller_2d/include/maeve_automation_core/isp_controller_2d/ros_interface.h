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
#include <std_msgs/Header.h>

#include <string>
#include <unordered_map>

#include "controller_interface_msgs/Command2D.h"
#include "maeve_automation_core/isp_controller_2d/control_command.h"
#include "maeve_automation_core/isp_controller_2d/isp_controller_2d.h"
#include "maeve_automation_core/maeve_geometry/interval.h"
#include "maeve_automation_core/ros_parameter_loading/params_base.h"

namespace maeve_automation_core {
/**
 * @brief Serialize and return this object.
 *
 * @return The serialized version of this object.
 */
controller_interface_msgs::Command2D controlCommand2Command2D_Msg(
    const ControlCommand& cmd, const std_msgs::Header& header);

/**
 * @brief Deserialize and return a control command object.
 *
 * @param msg The serialized version.
 *
 * @return The deserialized version.
 */
ControlCommand command2D_Msg2ControlCommand(
    const controller_interface_msgs::Command2D& msg);

/**
 * @brief Convenience method for filling a parameter object from the ROS
 * parameter server.
 *
 * @note Shape parameters for the ISP controller must live in a namespace
 * 'shape_parameters' that is relative to ns.
 *
 * @param nh ROS node handle.
 * @param ns The node-relative namespace that contains the controller params.
 * @param isp_controller_params The parameter object to fill out.
 *
 * @return True if 'isp_controller_params' was successfully filled out;
 * otherwise false.
 */
bool loadISP_ControllerROS_Params(const ros::NodeHandle& nh,
                                  const std::string& ns,
                                  ISP_Controller2D::Params& params);

/**
 * @brief Helper class for visualizing control horizons.
 */
class HorizonVisualizer {
 public:
  /**
   * @brief Parameter container for horizon visualizer.
   */
  struct Params {
    /** @brief Pixel height of the horizon visualization. */
    int horizon_viz_height;
    /** @brief Range for constraint values. */
    Interval_d constraint_range;

    /**
     * @brief Whether the parameter object has reasonable values.
     *
     * @return True if the values are reasonable; otherwise false.
     */
    bool valid() const;

    /**
     * @brief Constructor: initialize to invalid values.
     */
    Params();

    /**
     * @brief Explicit constructor.
     *
     * @param viz_height Height of visualization to generate.
     * @param r_min Constraint range min.
     * @param r_max Constraint range max.
     */
    Params(const int viz_height, const double r_min, const double r_max);
  };  // struct Params

  /**
   * @brief Initialize the visualization object.
   *
   * @param params Parameter set for the visualizer.
   * @param horizons The set of horizons to generalize visualizations for.
   * @param it The parent node's image transport object.
   */
  void initialize(const Params& params,
                  const std::vector<std::string>& horizons,
                  image_transport::ImageTransport& it);

  /**
   * @brief Generate and publish visualizations for the horizons.
   *
   * @param header The message header to use.
   * @param controller The ISP controller that holds the horizons.
   */
  void visualize(const std_msgs::Header& header,
                 const ISP_Controller2D& controller) const;

 private:
  /** @brief Parameter object for visualizer. */
  Params params_;

  /** @brief Horizon visualization publishers. */
  std::unordered_map<std::string, image_transport::Publisher> viz_horizon_pubs_;

  /** @brief Whether to use [0, 1] as the horizon bounds given the type. */
  static bool unit_visualization_bounds(
      const ISP_Controller2D::HorizonType& ht);

  /**
   * @brief Helper function for visualizing horizon structures.
   *
   * @param header Message header for published horizon visualization.
   * @param horizon The horizon to visualize.
   * @param ht The horizon type enum.
   * @param publisher The publisher to use to publish the visualization.
   */
  void visualizeHorizon(const std_msgs::Header& header, const cv::Mat& horizon,
                        const ISP_Controller2D::HorizonType ht,
                        const image_transport::Publisher& publisher) const;
};  // class HorizonVisualizer
}  // namespace maeve_automation_core
