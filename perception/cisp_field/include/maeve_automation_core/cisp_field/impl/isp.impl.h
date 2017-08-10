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

#include <algorithm>
#include <vector>

template <typename T_Tx>
ImageSpacePotentialField<T_Tx>::ImageSpacePotentialField(
    const cv::Mat& ttc_field, const T_Tx& tx)
    : tx_(tx) {
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
    default:
      // Unhandled input type. Cannot initialize.
      throw ISPInvalidInputTypeExcpetion();
  }

  // Perform transform.
  ApplyTransform transform(field_, tx_);
  cv::parallel_for_(cv::Range(0, field_.rows * field_.cols), transform);
}

template <typename T_Tx>
const char* ImageSpacePotentialField<T_Tx>::ISPInvalidInputTypeExcpetion::what()
    const noexcept {
  return "Image Space Potential initialization failed: Invalid input type. "
         "Must be CV_64F or CV_64FC2";
}

template <typename T_Tx>
const cv::Mat& ImageSpacePotentialField<T_Tx>::ImageSpacePotentialField::field()
    const {
  return field_;
}

template <typename T_Tx>
ImageSpacePotentialField<T_Tx>::ApplyTransform::ApplyTransform(cv::Mat& field,
                                                               const T_Tx& tx)
    : field_ref_(field), tx_(tx) {}

template <typename T_Tx>
void ImageSpacePotentialField<T_Tx>::ApplyTransform::operator()(
    const cv::Range& range) const {
  for (auto r = range.start; r != range.end; r++) {
    // Compute pixel indices.
    const auto row = r / field_ref_.cols;
    const auto col = r % field_ref_.cols;

    // Compute pixel -> potential transform.
    const auto p = field_ref_.ptr<double>(row)[2 * col];
    const auto p_dot = field_ref_.ptr<double>(row)[2 * col + 1];
    const auto potential_value = tx_(cv::Scalar(p, p_dot));

    // Set value.
    field_ref_.ptr<double>(row)[2 * col] = potential_value[0];
    field_ref_.ptr<double>(row)[2 * col + 1] = potential_value[1];
  }
}
