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
#include "karlsruhe_dataset_publisher/karlsruhe_dataset_publisher.h"

#include <limits>

namespace maeve_automation_core {
static constexpr auto NaN = std::numeric_limits<double>::quiet_NaN();

insdataRow insdataRow::createInsdataRow(const uint32_t _sec,
                                        const uint32_t _nsec, const double _lat,
                                        const double _lon, const double _alt,
                                        const double _x, const double _y,
                                        const double _z, const double _roll,
                                        const double _pitch,
                                        const double _yaw) {
  static constexpr auto invalid_roll = NaN;
  static constexpr auto invalid_pitch = NaN;
  return insdataRow(_sec, _nsec, _lat, _lon, _alt, _x, _y, _z, invalid_roll,
                    invalid_pitch, _yaw);
}

//------------------------------------------------------------------------------

sensor_msgs::NavSatFix insdataRow::convertToNavSatFix(
    const insdataRow& row, const std::string& frame_id) {
  sensor_msgs::NavSatFix msg;
  msg.header = insdataRow::getNavSatFixHeader(row, frame_id);
  msg.status = insdataRow::getNavSatFixStatus();
  msg.latitude = row.lat;
  msg.longitude = row.lon;
  msg.altitude = row.alt;
  msg.position_covariance_type =
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  std::fill(std::begin(msg.position_covariance),
            std::end(msg.position_covariance), NaN);
  return msg;
}

//------------------------------------------------------------------------------

std_msgs::Header insdataRow::getNavSatFixHeader(const insdataRow& row,
                                                const std::string& frame_id) {
  std_msgs::Header header;
  header.stamp = ros::Time(row.sec, row.nsec);
  header.frame_id = frame_id;
  return header;
}

//------------------------------------------------------------------------------

sensor_msgs::NavSatStatus insdataRow::getNavSatFixStatus() {
  sensor_msgs::NavSatStatus status;
  status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  return status;
}

}  // namespace maeve_automation_core
