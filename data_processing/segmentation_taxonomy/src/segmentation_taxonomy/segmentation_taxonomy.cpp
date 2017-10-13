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
const int SegmentationTaxonomy::INVALID_ID = 0;

SegmentationTaxonomy::Label::Label() : label_id(INVALID_ID) {}

bool SegmentationTaxonomy::load(const std::string& label_map_path,
                                const std::string& data_set_name) {
  LabelClasses label_classes;
  LabelInstances label_instances;
  std::tie(label_classes, label_instances) =
      loadLabels(label_map_path, data_set_name);
  return true;
}

const SegmentationTaxonomy::LabelMap& SegmentationTaxonomy::classMap() const {
  return class_map_;
}

const SegmentationTaxonomy::InstanceMap& SegmentationTaxonomy::instanceMap()
    const {
  return instance_map_;
}

int SegmentationTaxonomy::cvVec3bToInt(const cv::Vec3b& v) {
  return static_cast<int>(v[0]) + (255 * static_cast<int>(v[1])) +
         (255 * 255 * static_cast<int>(v[2]));
}
}  // namespace maeve_automation_core
