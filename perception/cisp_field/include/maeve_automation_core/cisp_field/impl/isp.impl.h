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

//
// DO NOT INCLUDE THIS FILE DIRECTLY. INCLUDE isp.h INSTEAD.
//

template <typename T_Tx>
cv::Mat ImageSpacePotentialField::build(const cv::Mat& measurement_field,
                                        const T_Tx& tx) {
  // Construct ISP.
  cv::Mat potential_field;
  potential_field = cv::Mat::zeros(0, 0, CV_64FC2);
  switch (measurement_field.type()) {
    case CV_64FC2: {
      measurement_field.copyTo(potential_field);
      break;
    }
    // case CV_64F: // This resolves to CV_64FC1
    case CV_64FC1: {
      // If no derivative information is included, assume 0.
      std::vector<cv::Mat> channels(2);
      channels[0] = potential_field;
      channels[1] = cv::Mat::zeros(measurement_field.rows,
                                   measurement_field.cols, CV_64F);

      cv::Mat tmp;
      cv::merge(channels, tmp);
      potential_field = tmp;
      break;
    }
    default: {
      // Unhandled input type. Cannot initialize. Leave field empty.
    }
  }

  // Perform transform.
  if (!potential_field.empty()) {
    ImageSpacePotentialField::ApplyTransform<T_Tx> apply(potential_field, tx);
    cv::parallel_for_(cv::Range(0, potential_field.rows * potential_field.cols),
                      apply);
  }

  // Done.
  return potential_field;
}

template <typename A_Tx>
ImageSpacePotentialField::ApplyTransform<A_Tx>::ApplyTransform(cv::Mat& field,
                                                               const A_Tx& tx)
    : field_ref_(field), tx_(tx) {}

template <typename A_Tx>
void ImageSpacePotentialField::ApplyTransform<A_Tx>::operator()(
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
