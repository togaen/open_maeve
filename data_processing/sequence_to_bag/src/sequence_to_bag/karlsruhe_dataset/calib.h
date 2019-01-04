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
#pragma once

#include <sensor_msgs/CameraInfo.h>

namespace maeve_automation_core {
namespace karlsruhe_dataset {

/**
 * @brief Holds the data contained in one calib.txt file
 *
 * This is intended strictly as an intermediary between the data set and message
 * objects. Only the static methods should ever have access to data members.
 */
struct calib {
  struct stereoCameraInfo {
    const sensor_msgs::CameraInfo left;
    const sensor_msgs::CameraInfo right;
    stereoCameraInfo(sensor_msgs::CameraInfo _left,
                     sensor_msgs::CameraInfo _right);
  };

  /**
   * @brief Get the CameraInfo message corresponding to the given calib text
   *
   * @note An exception is thrown if the text is malformed
   */
  static stereoCameraInfo convertToCameraInfo(const std_msgs::Header& header,
                                              const std::string& text,
                                              const int image_width,
                                              const int image_height);

 private:
  static constexpr auto ROW_DELIMITER = ' ';
  static constexpr auto P1_roi_prefix = "P1_roi";
  static constexpr auto P2_roi_prefix = "P2_roi";
  const boost::array<double, 12> P1_roi;
  const boost::array<double, 9> K1;
  const boost::array<double, 12> P2_roi;
  const boost::array<double, 9> K2;

  /**
   * @brief Create a calib object from the text of a calib.txt file
   *
   * @note Only the P1_roi and P2_roi data are captured because the images are
   * already rectified (see link to dataset description in README.md)
   *
   * @note An exception is thrown if the text is malformed
   */
  static calib createCalib(const std::string& text);

  /** @brief Explicit constructor */
  calib(boost::array<double, 12> _P1_roi, boost::array<double, 9> _K1,
        boost::array<double, 12> _P2_roi, boost::array<double, 9> _K2);
};  // struct calib

}  // namespace karlsruhe_dataset
}  // namespace maeve_automation_core
