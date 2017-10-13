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

#include <opencv2/opencv.hpp>

#include <string>
#include <unordered_map>

namespace maeve_automation_core {
/**
 * @brief Struct to capture segmentation label information.
 */
class SegmentationTaxonomy {
 public:
  /** @brief Constant to denote invalid identifiers. */
  static const int INVALID_ID;

  /**
   * @brief A label (class or instance) in the segmentation taxonomy.
   */
  struct Label {
    /** @brief Integer object class. */
    int label_id;
    /** @brief The RGB specification of class/id. */
    cv::Vec3b label_raw;
    /** @brief Name of this label. */
    std::string label_name;
    /**
     * @brief Default constructor: initialize to bad values.
     */
    Label();
  };  // struct Label

  /** @brief Typedef for the label map: Label Id -> Label */
  typedef std::unordered_map<int, Label> LabelMap;
  /** @brief Typedef for the instance label map: Class Id -> Instances */
  typedef std::unordered_map<int, LabelMap> InstanceMap;

  /**
   * @brief Load a label map from a YAML file.
   *
   * @param label_map_path The path to the label map file.
   * @param data_set_name The name of the data set being loaded.
   */
  __attribute__((warn_unused_result)) bool load(
      const std::string& label_map_path, const std::string& data_set_name);

  /**
   * @brief Const ref accessor to the internal class map.
   *
   * @return A const ref to the class map.
   */
  const LabelMap& classMap() const;

  /**
   * @brief Const ref accessor to the internal label map.
   *
   * @return A const ref to the instance map.
   */
  const InstanceMap& instanceMap() const;

 private:
  /** @brief Internal taxonomy data structure containing classes. */
  LabelMap class_map_;
  /** @brief Internal taxonomy data structure containing instances. */
  InstanceMap instance_map_;
  /**
   * @brief Convert a 3-tuple of type uchar to an integer.
   *
   * @param v The 3-tuple of type uchar.
   *
   * @return The integer value of three digit base 255 cv::Vec3b object.
   */
  static int cvVec3bToInt(const cv::Vec3b& v);
};  // class SegmentationTaxonomy
}  // namespace maeve_automation_core
