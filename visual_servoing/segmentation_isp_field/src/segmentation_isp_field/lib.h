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

#include "maeve_automation_core/segmentation_taxonomy/types.h"

namespace maeve_automation_core {
/**
 * @brief Compute an ISP field where all pixels of value 'label' in
 * 'segmentation' are set to 'potential_value'.
 *
 * @param segmentation The BGR segmentation image.
 * @param label_range The BGR value range to extract into an ISP field.
 * @param potential_value The potential value to assign to the ISP field.
 *
 * @return The extracted ISP field.
 */
cv::Mat extractGuidanceField(const cv::Mat& segmentation,
                             const LabelRange& label_range,
                             const cv::Point2d& potential_value);
}  // namespace maeve_automation_core
