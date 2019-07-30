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

#include "maeve_core/feature_flow/feature_flow.h"

#include "maeve_core/ros_parameter_loading/ros_parameter_loading.h"

namespace maeve_core {

/** @brief Parameter object to load ROS params.*/
struct FeatureFlowParams : public ParamsBase {
  /** @brief Feature Flow parameters. */
  FeatureFlow::Params ff;

  /** @brief Artificially reduce the frame rate of incoming video. */
  int skip_frames;

  /** @brief Ignore homographies within this distance of identity. */
  double identity_threshold;

  /** @brief Ignore homographies within this magnitude of scaling. */
  double scale_threshold;

  /** @brief Ignore homographies within this magnitude of translation. */
  double translation_threshold;

  /** @brief The camera image topic. */
  std::string camera_topic;

  /** @brief The visualization topic. */
  std::string viz_topic;

  /**
   * @copydoc ParamsBase::ParamsBase()
   */
  __attribute__((warn_unused_result)) bool load(
      const ros::NodeHandle& nh) override;
};  // struct FeatureFlowParams

}  // namespace maeve_core
