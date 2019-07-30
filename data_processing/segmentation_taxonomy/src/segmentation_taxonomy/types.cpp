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
#include "maeve_core/segmentation_taxonomy/types.h"

#include <algorithm>

namespace maeve_core {
std::ostream& operator<<(std::ostream& os, const LabelRange& label_range) {
  return os << "[" << std::get<0>(label_range) << ", "
            << std::get<1>(label_range) << "]";
}

std::ostream& operator<<(std::ostream& os, const LabelClasses& label_classes) {
  std::for_each(std::begin(label_classes), std::end(label_classes),
                [&](const LabelClasses::value_type& p) {
                  os << p.first << ": " << p.second << " ";
                });
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const LabelInstances& label_instances) {
  std::for_each(std::begin(label_instances), std::end(label_instances),
                [&](const LabelInstances::value_type& v) { os << v << "\n"; });
  return os;
}

std::ostream& operator<<(std::ostream& os, const ClassSet& class_set) {
  std::for_each(std::begin(class_set), std::end(class_set),
                [&](const ClassSet::value_type& s) { os << s << " "; });
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const LabelInstanceClasses& label_instance_classes) {
  std::for_each(
      std::begin(label_instance_classes), std::end(label_instance_classes),
      [&](const LabelInstanceClasses::value_type& v) { os << v << "\n"; });
  return os;
}

bool labelRangeValid(const LabelRange& label_range) {
  const auto& rgb_min = std::get<0>(label_range);
  const auto& rgb_max = std::get<1>(label_range);
  for (auto i = 0; i < 3; ++i) {
    if (rgb_min[i] > rgb_max[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace maeve_core
