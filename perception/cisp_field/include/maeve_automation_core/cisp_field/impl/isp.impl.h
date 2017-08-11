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
ImageSpacePotentialField ImageSpacePotentialField::build(
    const cv::Mat& ttc_field, const T_Tx& tx) {
  // Construct ISP.
  ImageSpacePotentialField isp(ttc_field);

  // Perform transform.
  ImageSpacePotentialField::ApplyTransform<T_Tx> apply(isp.field_, tx);
  cv::parallel_for_(cv::Range(0, isp.field_.rows * isp.field_.cols), apply);

  // Done.
  return isp;
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
