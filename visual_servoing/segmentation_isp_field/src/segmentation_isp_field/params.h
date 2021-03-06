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

#include <string>
#include <vector>

#include "open_maeve/isp_controller_2d/isp_controller_2d.h"
#include "open_maeve/maeve_geometry/interval.h"
#include "open_maeve/ros_parameter_loading/ros_parameter_loading.h"

namespace open_maeve {
/** @brief Parameter object to load ROS params.*/
struct SegmentationFieldParams : public ParamsBase {
  /** @brief ISP controller parameters. */
  ISP_Controller2D::Params isp_controller_params;

  /** @brief Guidance control to use absent explicit user input. */
  ControlCommand default_guidance_control;

  /** @brief The image sequence topic. */
  std::string segmentation_sequence_topic;

  /** @brief The ISP field visualization topic. */
  std::string viz_isp_field_topic;

  /** @brief The horizon structures to visualize. */
  std::vector<std::string> visualize_horizons;

  /** @brief Control command output topic. */
  std::string control_command_output_topic;

  /** @brief Desired control command input topic. */
  std::string control_command_input_topic;

  /** @brief Path to the label map file. */
  std::string segmentation_taxonomy;

  /** @brief Name of the data set to load a taxonomy for. */
  std::string data_set_name;

  /** @brief The scaling bounds for visualizing potential values. */
  Interval_d viz_potential_bounds;

  /** @brief Height of horizon visualization. */
  int horizon_viz_height;

  /** @brief Hard constraint transform parameters. */
  ShapeParameters hard_constraint_transform;

  /** @brief Soft constraint transform parameters. */
  ShapeParameters soft_constraint_transform;

  /**
   * @brief Check validity of loaded parameter values.
   *
   * @return True if values valid; otherwise false.
   */
  __attribute__((warn_unused_result)) bool valid() const;

  /**
   * @copydoc ParamsBase::ParamsBase()
   */
  __attribute__((warn_unused_result)) bool load(
      const ros::NodeHandle& nh) override;

 private:
  /** @brief The scaling bounds for visualizing potential values. */
  std::vector<double> viz_potential_bounds_;
};  // struct SegmentationFieldParams

}  // namespace open_maeve
