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
#include "maeve_automation_core/segmentation_taxonomy/segmentation_taxonomy.h"

#include <array>
#include <vector>

#include "segmentation_taxonomy/io.h"

namespace maeve_automation_core {
bool SegmentationTaxonomy::load(const std::string& label_map_path,
                                const std::string& data_set_name) {
  // Retrieve information from yaml file.
  std::tie(classes, instances, instance_classes) =
      loadLabels(label_map_path, data_set_name);

  // All good.
  return valid();
}

bool SegmentationTaxonomy::valid() const {
  // There must be at least one class.
  if (classes.empty()) {
    return false;
  }

  // Instances and instance classes must be lists of the same size.
  if (instances.size() != instance_classes.size()) {
    return false;
  }

  // Each instance must have at least one class.
  for (const auto& s : instance_classes) {
    if (s.empty()) {
      return false;
    }

    // Each instance class must exist in the class set.
    for (const auto& cs : s) {
      if (classes.find(cs) == classes.end()) {
        return false;
      }
    }
  }

  // All checks pass.
  return true;
}

int SegmentationTaxonomy::cvVec3bToInt(const cv::Vec3b& v) {
  return static_cast<int>(v[0]) + (255 * static_cast<int>(v[1])) +
         (255 * 255 * static_cast<int>(v[2]));
}
}  // namespace maeve_automation_core
