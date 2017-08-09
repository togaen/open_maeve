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

template <typename T_TxK0>
ImageSpacePotentialField<T_TxK0>::ImageSpacePotentialField(
    const cv::Mat& ttc_field, const T_Tx& tx)
    : field_(ttc_field.rows, ttc_field.cols, CV_64FC2, cv::Scalar(0.0, 0.0)),
      tx_(tx) {
  // Transform ttc_field into ISP.
  switch (ttc_field.type()) {
    case CV_64FC2:
    // Perform C1 transform, fall through to perform C0.
    case CV_64F:
    case CV_64FC1:
      // Perform C0 transform.
      break;
    default:
      // Unhandled input type. Cannot initialize.
      throw ISPInvalidInputTypeExcpetion();
  }
}

template <typename T_Tx>
const char* ImageSpacePotentialField<T_Tx>::ISPInvalidInputTypeExcpetion::what()
    const {
  return "Image Space Potential initialization failed: Invalid input type. "
         "Must be CV_64F or CV_64FC2";
}

template <typename T_Tx>
ImageSpacePotentialField<T_Tx>::ImageSpacePotentialField::field() const {
  return field_;
}

template <typename T_Tx>
ImageSpacePotentialField<T_Tx>::ApplyTransform::ApplyTransform(cv::Mat& field,
                                                               const T_Tx& tx)
    : field_ref_(field), tx_(tx) {}

template <typename T_Tx>
void ImageSpacePotentialField<T_Tx>::ApplyTransform::operator()(
    const cv::Range& r) const override {
  for (auto r = r.start; r != r.end; r++) {
    // Compute pixel indices.
    const auto i = r / field_ref_.cols;
    const auto j = r % field_ref_.cols;

    // Compute pixel -> potential transform.
    const auto p = field_ref_.ptr<double>(i)[j];
    const auto p_dot = field_ref_.ptr<double>(i)[j + 1];
    const auto potential_value = tx_(cv::Scalar(p, p_dot));

    // Set value.
    field_ref_.ptr<double>(i)[j] = potential_value;
  }
}
