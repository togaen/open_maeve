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

#include <iostream>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace maeve_automation_core {
/** @brief Color value range to associate with a label. */
typedef std::tuple<cv::Vec3b, cv::Vec3b> LabelRange;
/** @brief Map of string class name to color value range. */
typedef std::unordered_map<std::string, LabelRange> LabelClasses;
/** @brief Ordered set of instance label values. */
typedef std::vector<LabelRange> LabelInstances;
/** @brief Set of string class names. */
typedef std::set<std::string> ClassSet;
/** @brief Ordered set of instance class sets. */
typedef std::vector<ClassSet> LabelInstanceClasses;

/**
 * @brief Whether a label range exhibits proper ordering.
 *
 * @param label_range The label range to validate.
 *
 * @return True if the range is proper; otherwise false.
 */
bool labelRangeValid(const LabelRange& label_range);

/**
 * @brief Stream overload for LabelRange.
 *
 * @param os The output stream.
 * @param label_range The label range object.
 *
 * @return The serialized label range.
 */
std::ostream& operator<<(std::ostream& os, const LabelRange& label_range);

/**
 * @brief Stream overload for LabelClasses.
 *
 * @param os The output stream.
 * @param label_range The label class object.
 *
 * @return The serialized label class.
 */
std::ostream& operator<<(std::ostream& os, const LabelClasses& label_classes);

/**
 * @brief Stream overload for LabelInstances.
 *
 * @param os The output stream.
 * @param label_range The label instances object.
 *
 * @return The serialized label instances.
 */
std::ostream& operator<<(std::ostream& os,
                         const LabelInstances& label_instances);

/**
 * @brief Stream overload for ClassSet.
 *
 * @param os The output stream.
 * @param label_range The class set object.
 *
 * @return The serialized class set.
 */
std::ostream& operator<<(std::ostream& os, const ClassSet& class_set);

/**
 * @brief Stream overload for LabelInstanceClasses.
 *
 * @param os The output stream.
 * @param label_instance_classes The class set object.
 *
 * @return The serialized label instance classes.
 */
std::ostream& operator<<(std::ostream& os,
                         const LabelInstanceClasses& label_instace_classes);
}  // namespace maeve_automation_core
