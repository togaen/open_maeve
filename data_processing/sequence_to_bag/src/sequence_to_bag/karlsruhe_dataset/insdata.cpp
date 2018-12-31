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
#include "sequence_to_bag/karlsruhe_dataset/insdata.h"

#include "sequence_to_bag/sequence_to_bag.h"

#include <limits>
#include <sstream>
#include <stdexcept>

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

namespace maeve_automation_core {
namespace karlsruhe_dataset {

static constexpr auto NaN = std::numeric_limits<double>::quiet_NaN();

//------------------------------------------------------------------------------

insdataRow::insdataRow(const uint32_t _sec, const uint32_t _nsec,
                       const double _lat, const double _lon, const double _alt,
                       const double _x, const double _y, const double _z,
                       const double _roll, const double _pitch,
                       const double _yaw)
    : sec(_sec),
      nsec(_nsec),
      lat(_lat),
      lon(_lon),
      alt(_alt),
      x(_x),
      y(_y),
      z(_z),
      roll(NaN),
      pitch(NaN),
      yaw(_yaw)

{}

//------------------------------------------------------------------------------

std_msgs::Header insdataRow::getStampedHeader(const std::string& row_text) {
  const auto row = createInsdataRow(row_text);
  return getHeader(row, "");
}

//------------------------------------------------------------------------------

insdataRow::GPS_IMU insdataRow::convertToGPS_IMU(const std::string& row_text,
                                                 const std::string& frame_id) {
  const auto row = createInsdataRow(row_text);
  return {convertToNavSatFix(row, frame_id), convertToOdometry(row, frame_id)};
}

//------------------------------------------------------------------------------

sensor_msgs::NavSatFix insdataRow::convertToNavSatFix(
    const insdataRow& row, const std::string& frame_id) {
  sensor_msgs::NavSatFix msg;
  msg.header = insdataRow::getHeader(row, frame_id);
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

nav_msgs::Odometry insdataRow::convertToOdometry(const insdataRow& row,
                                                 const std::string& frame_id) {
  nav_msgs::Odometry msg;
  msg.header = getHeader(row, frame_id);
  msg.child_frame_id = frame_id;

  geometry_msgs::Point position;
  position.x = row.x;
  position.y = row.y;
  position.z = row.z;
  geometry_msgs::Pose P;
  P.position = position;
  P.orientation = tf::createQuaternionMsgFromYaw(row.yaw);

  geometry_msgs::PoseWithCovariance P_C;
  P_C.pose = P;
  std::fill(std::begin(P_C.covariance), std::end(P_C.covariance), NaN);

  geometry_msgs::Vector3 invalid_velocity;
  invalid_velocity.x = NaN;
  invalid_velocity.y = NaN;
  invalid_velocity.z = NaN;
  geometry_msgs::Twist T;
  T.linear = invalid_velocity;
  T.angular = invalid_velocity;

  geometry_msgs::TwistWithCovariance T_C;
  T_C.twist = T;
  std::fill(std::begin(T_C.covariance), std::end(T_C.covariance), NaN);

  msg.pose = P_C;
  msg.twist = T_C;

  return msg;
}

//------------------------------------------------------------------------------

std_msgs::Header insdataRow::getHeader(const insdataRow& row,
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

//------------------------------------------------------------------------------

insdataRow insdataRow::createInsdataRow(const std::string& row_text) {
  const auto tokens = stringSplit(row_text, ROW_DELIMITER);

  if (tokens.size() != ROW_TOKEN_COUNT) {
    std::stringstream ss;
    ss << "Error parsing insdata row string \"" << row_text
       << "\": expected exactly " << ROW_TOKEN_COUNT << " tokens, got "
       << tokens.size();
    throw std::invalid_argument(ss.str());
  }

  try {
    uint32_t sec = 0;
    uint32_t nsec = 0;
    std::tie(sec, nsec) = parseTime(tokens[0]);

    const auto lat = std::stod(tokens[1]);
    const auto lon = std::stod(tokens[2]);
    const auto alt = std::stod(tokens[3]);
    const auto x = std::stod(tokens[4]);
    const auto y = std::stod(tokens[5]);
    const auto z = std::stod(tokens[6]);
    const auto roll = std::stod(tokens[7]);
    const auto pitch = std::stod(tokens[8]);
    const auto yaw = std::stod(tokens[9]);

    return insdataRow(sec, nsec, lat, lon, alt, x, y, z, roll, pitch, yaw);
  } catch (const std::exception& e) {
    std::stringstream ss;
    ss << "Error parsing one or more of the following values: ";
    std::for_each(std::begin(tokens), std::end(tokens),
                  [&ss](const std::string& val) { ss << val << " "; });
    ss << "\n" << e.what();
    throw std::invalid_argument(ss.str());
  }
}

//------------------------------------------------------------------------------

std::tuple<uint32_t, uint32_t> insdataRow::parseTime(
    const std::string& time_text) {
  if (time_text.length() != TIMESTAMP_DIGITS) {
    std::stringstream ss;
    ss << "Timestamp has invalid length (must be " << TIMESTAMP_DIGITS << ")";
    throw std::invalid_argument(ss.str());
  }

  const auto sec_string = time_text.substr(0, 10);
  const auto nsec_string = time_text.substr(11, 9);
  const uint32_t sec = std::stoul(sec_string);
  const uint32_t nsec = std::stoul(nsec_string);
  return std::make_tuple(sec, nsec);
}

}  // namespace karlsruhe_dataset
}  // namespace maeve_automation_core
