/*
 * Copyright 2018 Maeve Automation
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
#include "sequence_to_bag/karlsruhe_dataset/calib.h"

#include "sequence_to_bag/sequence_to_bag.h"

#include <sstream>
#include <stdexcept>

namespace maeve_automation_core {
namespace karlsruhe_dataset {

calib::calib(boost::array<double, 12> _P1_roi, boost::array<double, 9> _K1,
             boost::array<double, 12> _P2_roi, boost::array<double, 9> _K2)
    : P1_roi(_P1_roi), K1(_K1), P2_roi(_P2_roi), K2(_K2) {}

//------------------------------------------------------------------------------

calib::stereoCameraInfo calib::convertToCameraInfo(
    const std_msgs::Header& header, const std::string& text,
    const int image_width, const int image_height) {
  const auto c = createCalib(text);

  auto camera_info =
      getUndistortedCameraInfo(header, image_width, image_height);
  stereoCameraInfo stereo_camera_info = {camera_info, camera_info};

  stereo_camera_info.left.P = c.P1_roi;
  stereo_camera_info.left.K = c.K1;
  stereo_camera_info.right.P = c.P2_roi;
  stereo_camera_info.right.K = c.K2;

  return stereo_camera_info;
}

//------------------------------------------------------------------------------

calib calib::createCalib(const std::string& text) {
  const auto P1_roi_opt = getPrefixedRow(P1_roi_prefix, text);
  const auto P2_roi_opt = getPrefixedRow(P1_roi_prefix, text);
  if (!P1_roi_opt || !P2_roi_opt) {
    std::stringstream ss;
    ss << "At least one of '" << P1_roi_prefix << "' or '" << P2_roi_prefix
       << "' not found in calib text:\n"
       << text;
    throw std::invalid_argument(ss.str());
  }

  const auto P1_roi_tokens = stringSplit(*P1_roi_opt, ROW_DELIMITER);
  const auto P2_roi_tokens = stringSplit(*P2_roi_opt, ROW_DELIMITER);
  if ((P1_roi_tokens.size() != 12) || (P2_roi_tokens.size() != 12)) {
    std::stringstream ss;
    ss << "Found " << P1_roi_tokens.size() << " tokens in the P1_roi row and "
       << P2_roi_tokens.size()
       << " tokens in the P2_roi row, but both rows are required to have "
          "exactly "
       << 12 << " tokens";
    throw std::invalid_argument(ss.str());
  }

  const auto string_to_double = [](const std::string& str) {
    return std::stod(str);
  };

  const auto get_3x3_from_3x4 = [](const boost::array<double, 12>& M) {
    boost::array<double, 9> A;
    for (auto i = 0; i < 3; ++i) {
      for (auto j = 0; j < 3; ++j) {
        const auto A_idx = ((i * 3) + j);
        const auto M_idx = ((i * 4) + j);
        A[A_idx] = M[M_idx];
      }
    }
    return A;
  };

  boost::array<double, 12> P1_roi;
  std::transform(std::begin(P1_roi_tokens), std::end(P1_roi_tokens),
                 std::begin(P1_roi), string_to_double);
  const auto K1 = get_3x3_from_3x4(P1_roi);
  boost::array<double, 12> P2_roi;
  std::transform(std::begin(P2_roi_tokens), std::end(P2_roi_tokens),
                 std::begin(P2_roi), string_to_double);
  const auto K2 = get_3x3_from_3x4(P2_roi);

  return calib(P1_roi, K1, P2_roi, K2);
}

}  // namespace karlsruhe_dataset
}  // namespace maeve_automation_core
