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
#include <tuple>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

namespace maeve_automation_core {
namespace karlsruhe_dataset {

/**
 * @brief Holds one row of data from an insdata.txt file
 *
 * This is intended strictly as an intermediary between the data set and message
 * objects. Only the static methods should ever have access to data members.
 */
struct insdataRow {
  struct GPS_IMU {
    sensor_msgs::NavSatFix gps;
    nav_msgs::Odometry imu;
  };  // struct

  /**
   * @brief Get a header with the timestamp from the given row of the insdata
   * file
   *
   * @post The returned header only has its stamp field set; other fields are
   * uninitializaed
   */
  static std_msgs::Header getStampedHeader(const std::string& row_text);

  /**
   * @brief Construct and return the GPS and IMU messages for a given row
   *
   * @note An exception is thrown if the row text is malformed
   *
   * @note The roll and pitch values from the IMU are corrupt in the insdata.txt
   * files (see link to dataset description in README.md), therefore only yaw is
   * set and roll and pitch are zeroed
   *
   * @note insdata.txt files contain no covariance or velocity information,
   * therefore all such values are set to NaN
   */
  static GPS_IMU convertToGPS_IMU(const std::string& row_text,
                                  const std::string& frame_id);

 private:
  static constexpr auto ROW_DELIMITER = ' ';
  static constexpr auto ROW_TOKEN_COUNT = 10;
  static constexpr auto TIMESTAMP_DIGITS = 19;

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
   * @brief Explicit constructor
   *
   * @note The roll and pitch are replaced with NaN because the data in the file
   * are corrupt (see link to dataset description in README.md)
   */
  insdataRow(const uint32_t _sec, const uint32_t _nsec, const double _lat,
             const double _lon, const double _alt, const double _x,
             const double _y, const double _z, const double _roll,
             const double _pitch, const double _yaw);

  /**
   * @brief Parse and single row of space-delimited insdata text to create an
   * insdata object
   *
   * The columns are: 'timestamp lat lon alt x y z roll pitch yaw'
   *
   * @note An exception is thrown if the row text is malformed
   */
  static insdataRow createInsdataRow(const std::string& row_text);

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

  /**
   * @brief Build a ROS Odometry message containing the information from an
   * insdataRow object
   *
   * @note insdata.txt files contain no velocity information and corrupted roll
   * and pitch information, therefore only position and yaw are usable in the
   * Odometry message
   *
   * @note The odometer is treated as the base link of the robot, so it is its
   * own child frame
   */
  static nav_msgs::Odometry convertToOdometry(const insdataRow& row,
                                              const std::string& frame_id);

  /**
   * @brief Parse a 19-digit nanosecond time string to seconds and nanoseconds
   */
  static std::tuple<uint32_t, uint32_t> parseTime(const std::string& time_text);

  /**
   * @brief Store insdata row timestamp and given frame id in a header message
   */
  static std_msgs::Header getHeader(const insdataRow& row,
                                    const std::string& frame_id);

  /**
   * @brief Status is set to STATUS_FIX and service to SERVICE_GPS
   */
  static sensor_msgs::NavSatStatus getNavSatFixStatus();
};  // struct insdataRow

}  // namespace karlsruhe_dataset
}  // namespace maeve_automation_core
