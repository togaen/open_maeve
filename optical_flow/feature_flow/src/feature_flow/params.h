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

#include "maeve_automation_core/ros_parameter_loading/params_base.h"

#include <string>

namespace maeve_automation_core {

/** @brief Parameter object to load ROS params.*/
struct FeatureFlowParams : public ParamsBase {
  /** @brief Threshold for what constitutes a "good" match. */
  float good_match_portion;

  /** @name BRISK feature detection parameters
   * @{
   */
  /** @brief Threshold level. */
  int threshold_level;
  /** @brief Octaves. */
  int octaves;
  /** @brief Pattern scales. */
  float pattern_scales;
  /** @} */

  /** @name Locally Sensitive Hashing parameters
   * @{
   */
  /** @brief Number of hash tables to use. */
  int lsh_table_number;
  /** @brief Key bits to use. */
  int lsh_key_size;
  /** @brief Typically set to 2. */
  int lsh_multi_probe_level;
  /** @} */

  /** @brief The camera image topic. */
  std::string camera_topic;

  /**
   * @copydoc ParamsBase::ParamsBase()
   */
  __attribute__((warn_unused_result)) bool load(
      const ros::NodeHandle& nh) override;
};  // struct FeatureFlowParams

}  // namespace maeve_automation_core
