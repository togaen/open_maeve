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
#include "segmentation_taxonomy/io.h"

#include <yaml-cpp/yaml.h>
#include <boost/optional.hpp>

namespace maeve_automation_core {
namespace {
boost::optional<cv::Vec3b> yaml_rgb_to_cvvec3b(const YAML::Node& node) {
  if (!node.IsSequence() || (node.size() != 3)) {
    return boost::none;
  }

  cv::Vec3b bgr;
  auto idx = 2;  // Count backwards to convert RGB to BGR for OpenCV.
  for (auto it = node.begin(); it != node.end(); ++it, --idx) {
    // yaml-cpp won't allow casting directly to unsigned char.
    bgr[idx] = static_cast<unsigned char>(it->as<unsigned int>());
  }
  return bgr;
}
}  // namespace

std::tuple<LabelClasses, LabelInstances, LabelInstanceClasses> loadLabels(
    const std::string& label_map_path, const std::string& data_set_name) {
  LabelClasses classes;
  LabelInstances instances;
  LabelInstanceClasses instance_classes;

  // Load YAML file.
  YAML::Node config;
  try {
    config = YAML::LoadFile(label_map_path);
  } catch (...) {
    return std::make_tuple(classes, instances, instance_classes);
  }

  {  // Load classes.
    YAML::Node classes_node = config[data_set_name]["label_classes"];
    if (classes_node.IsMap()) {
      for (auto it = classes_node.begin(); it != classes_node.end(); ++it) {
        YAML::Node childKey = it->first;
        YAML::Node childValue = it->second;

        const auto class_name = childKey.as<std::string>();
        if (auto rgb = yaml_rgb_to_cvvec3b(childValue)) {
          classes[class_name] = *rgb;
        }
      }
    }
  }

  {  // Load instances.
    YAML::Node instances_node = config[data_set_name]["label_instances"];
    if (instances_node.IsSequence()) {
      for (auto it = instances_node.begin(); it != instances_node.end(); ++it) {
        if (auto rgb = yaml_rgb_to_cvvec3b(*it)) {
          instances.push_back(*rgb);
        }
      }
    }
  }

  {  // Load instance classes.
    YAML::Node instances_node = config[data_set_name]["label_instance_classes"];
    if (instances_node.IsSequence()) {
      for (auto it = instances_node.begin(); it != instances_node.end(); ++it) {
        if (it->IsSequence() && (it->size() > 0)) {
          ClassSet class_set;
          for (auto it_classes = it->begin(); it_classes != it->end();
               ++it_classes) {
            class_set.insert(it_classes->as<std::string>());
          }
          instance_classes.push_back(class_set);
        }
      }
    }
  }

  // Done.
  return std::make_tuple(classes, instances, instance_classes);
}
}  // namespace maeve_automation_core
