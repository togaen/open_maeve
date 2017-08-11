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
#include "maeve_automation_core/cisp_field/isp.h"

#include <vector>

namespace maeve_automation_core {
ImageSpacePotentialField::ImageSpacePotentialField(const cv::Mat& ttc_field) {
  // Copy ttc_field.
  switch (ttc_field.type()) {
    case CV_64FC2: {
      ttc_field.copyTo(field_);
      break;
    }
    // case CV_64F: // This resolves to CV_64FC1
    case CV_64FC1: {
      // If no derivative information is included, assume 0.
      std::vector<cv::Mat> channels(2);
      channels[0] = ttc_field;
      channels[1] = cv::Mat::zeros(ttc_field.rows, ttc_field.cols, CV_64F);

      cv::merge(channels, field_);
      break;
    }
    default: {
      // Unhandled input type. Cannot initialize.
      throw ISPInvalidInputTypeExcpetion();
    }
  }
}

const char* ImageSpacePotentialField::ISPInvalidInputTypeExcpetion::what() const
    noexcept {
  return "Image Space Potential initialization failed: Invalid input type. "
         "Must be CV_64F or CV_64FC2";
}

const cv::Mat& ImageSpacePotentialField::ImageSpacePotentialField::field()
    const {
  return field_;
}

}  // namespace maeve_automation_core
