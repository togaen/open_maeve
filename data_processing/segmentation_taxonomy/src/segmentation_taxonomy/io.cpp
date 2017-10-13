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

namespace maeve_automation_core {
std::tuple<LabelClasses, LabelInstances> loadLabels(
    const std::string& label_map_path, const std::string& data_set_name) {
  LabelClasses classes;
  LabelInstances instances;

  // Load YAML file.
  YAML::Node config;
  try {
    config = YAML::LoadFile(label_map_path);
  } catch (...) {
    return std::make_tuple(classes, instances);
  }

  {  // Load classes.
    YAML::Node classes_node = config[data_set_name]["label_classes"];
    if (classes_node.IsMap()) {
      for (auto it = classes_node.begin(); it != classes_node.end(); ++it) {
        YAML::Node childKey = it->first;
        YAML::Node childValue = it->second;

        const auto class_name = childKey.as<std::string>();
        if (childValue.IsSequence() && (childValue.size() == 3)) {
          RGB rgb;
          auto idx = 0;
          for (auto it_rgb = childValue.begin(); it_rgb != childValue.end();
               ++it_rgb, ++idx) {
            rgb[idx] = it_rgb->as<int>();
          }
          classes[class_name] = rgb;
        }
      }
    }
  }

  {  // Load instances.
    YAML::Node instances_node = config[data_set_name]["label_instances"];
    if (instances_node.IsSequence()) {
      for (auto it = instances_node.begin(); it != instances_node.end(); ++it) {
        if (it->IsSequence() && (it->size() == 3)) {
          RGB rgb;
          auto idx = 0;
          for (auto it_rgb = it->begin(); it_rgb != it->end();
               ++it_rgb, ++idx) {
            rgb[idx] = it_rgb->as<int>();
          }
          instances.push_back(rgb);
        }
      }
    }
  }

  // Done.
  return std::make_tuple(classes, instances);
}
}  // namespace maeve_automation_core
