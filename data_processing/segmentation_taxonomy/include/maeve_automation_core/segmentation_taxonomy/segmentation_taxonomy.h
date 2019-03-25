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

#include <iostream>
#include <string>

#include "maeve_automation_core/segmentation_taxonomy/types.h"

namespace maeve_automation_core {
/**
 * @brief Struct to capture segmentation label information.
 */
struct SegmentationTaxonomy {
  /** @brief Map of string name to RGB label value (cv::Vec3b) */
  LabelClasses classes;
  /** @brief Vector of RGB label instance IDs. */
  LabelInstances instances;
  /** @brief Vector of string class name sets that match instance IDs. */
  LabelInstanceClasses instance_classes;

  /**
   * @brief Load a label map from a YAML file.
   *
   * @param segmentation_taxonomy The path to the label map file.
   * @param data_set_name The name of the data set being loaded.
   */
  __attribute__((warn_unused_result)) bool load(
      const std::string& segmentation_taxonomy, const std::string& data_set_name);

  /**
   * @brief Whether the loaded parameters pass basic sanity checks.
   *
   * @return True if taxonomy information is consistent; otherwise false.
   */
  __attribute__((warn_unused_result)) bool valid() const;

  /**
   * @brief Convert a 3-tuple of type uchar to an integer.
   *
   * @param v The 3-tuple of type uchar.
   *
   * @return The integer value of three digit base 255 cv::Vec3b object.
   */
  static int cvVec3bToInt(const cv::Vec3b& v);
};  // class SegmentationTaxonomy

/**
 * @brief Stream overload to segmentation taxonomy struct.
 *
 * @param os The output stream.
 * @param t The segmentation taxonomy.
 *
 * @return The serialized segmentation taxonomy.
 */
std::ostream& operator<<(std::ostream& os, const SegmentationTaxonomy& t);
}  // namespace maeve_automation_core
