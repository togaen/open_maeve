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

#include <cstdint>

#include <sensor_msgs/NavSatFix.h>

namespace maeve_automation_core {

/**
 * @brief Holds one row of data from an insdata.txt file
 */
struct insdataRow {
  const uint32_t sec;
  const uint32_t nsec;
  const double lat;
  const double lon;
  const double alt;
  const double x;
  const double y;
  const double z;
  const double roll;
  const double pitch;
  const double yaw;

  /**
   * @brief Factory method for an insdataRow object.
   *
   * @note The roll and pitch are replaced with NaN because the data in the file
   * are corrupt (see link to dataset in README.md)
   */
  static insdataRow createInsdataRow(const uint32_t _sec, const uint32_t _nsec,
                                     const double _lat, const double _lon,
                                     const double _alt, const double _x,
                                     const double _y, const double _z,
                                     const double _roll, const double _pitch,
                                     const double _yaw);

  /**
   * @brief Build a ROS NavSatFix message containing the information from an
   * insdataRow object
   *
   * @note insdata.txt files contain no covariance information, therefore the
   * position covariance matrix is filled with NaN and covariance type is set to
   * UNKNOWN
   */
  static sensor_msgs::NavSatFix convertToNavSatFix(const insdataRow& row,
                                                   const std::string& frame_id);

 private:
  /**
   * @brief Store insdata row timestamp and given frame id in a header message
   */
  static std_msgs::Header getNavSatFixHeader(const insdataRow& row,
                                             const std::string& frame_id);

  /**
   * @brief Status is set to STATUS_FIX and service to SERVICE_GPS
   */
  static sensor_msgs::NavSatStatus getNavSatFixStatus();

  /**
   * @brief Explicit constructor
   */
  insdataRow(const uint32_t _sec, const uint32_t _nsec, const double _lat,
             const double _lon, const double _alt, const double _x,
             const double _y, const double _z, const double _roll,
             const double _pitch, const double _yaw);
};  // struct insdataRow

}  // namespace maeve_automation_core
