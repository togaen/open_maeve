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

#include "sequence_to_bag/karlsruhe_dataset/karlsruhe_dataset.h"
#include "sequence_to_bag/karlsruhe_dataset/calib.h"
#include "sequence_to_bag/karlsruhe_dataset/insdata.h"

#include <tf2_msgs/TFMessage.h>

namespace {
namespace po = boost::program_options;
}  // namespace

int main(int argc, char** argv) {
  boost::optional<std::string> data_set_path_opt;
  boost::optional<std::string> output_path_opt;
  constexpr auto CAMERA_NAME_DEFAULT = "stereo";
  constexpr auto GLOBAL_NAME_DEFAULT = "world";
  constexpr auto IMU_NAME_DEFAULT = "imu";

  po::options_description desc(
      "Karlsruhe Dataset sequencer. See README.md for details.\nAvailable "
      "options are listed below. Arguments without default "
      "values are required",
      open_maeve::PROGRAM_OPTIONS_LINE_LENGTH);
  desc.add_options()("help,h", "Print help and exit.")(
      "data-set-path,d", po::value(&data_set_path_opt)->required(),
      "Absolute path to the data set directory.")(
      "bag-output-dir,o", po::value(&output_path_opt)->required(),
      "Absolute path to the directory for the output bag file.")(
      "camera-name,c",
      po::value<std::string>()->default_value(CAMERA_NAME_DEFAULT),
      "Frame name to use for the stereo camera image stream.")(
      "global-name,g",
      po::value<std::string>()->default_value(GLOBAL_NAME_DEFAULT),
      "Name to use for the global frame in which the IMU measures "
      "motion.")("imu-name,i",
                 po::value<std::string>()->default_value(IMU_NAME_DEFAULT),
                 "Name to use for the local IMU frame as it moves in the "
                 "global frame.");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  const auto help_requested = vm.count("help");
  if (help_requested) {
    std::cout << desc << "\n";
    return EXIT_SUCCESS;
  }

  try {
    po::notify(vm);
  } catch (boost::program_options::required_option& e) {
    std::cerr << "Ensure that all required options are specified: " << e.what()
              << "\n\n";
    std::cerr << desc << "\n";
    return EXIT_FAILURE;
  }

  const auto data_set_path = *data_set_path_opt;
  const auto data_set_name =
      boost::filesystem::path(data_set_path).filename().string();
  const auto output_path = *output_path_opt;
  const auto camera_name = vm["camera-name"].as<std::string>();
  const auto global_name = vm["global-name"].as<std::string>();
  const auto imu_name = vm["imu-name"].as<std::string>();

  const auto calib_text =
      open_maeve::karlsruhe_dataset::getCalibText(data_set_path);
  const auto insdata_text =
      open_maeve::karlsruhe_dataset::getInsdataText(data_set_path);

  // Look at the first row of the insdata file to get timestamp information for
  // static transforms and camera info
  std::string first_row;
  std::istringstream ss(insdata_text);
  if (!std::getline(ss, first_row)) {
    std::cerr << "Failed to read first row of insdata.txt\n";
    return EXIT_FAILURE;
  }

  const auto stamped_header =
      open_maeve::karlsruhe_dataset::insdataRow::getStampedHeader(
          first_row);

  auto camera_header = stamped_header;
  camera_header.frame_id = camera_name;

  // Look at the first stereo image to get dimensions for camera info
  const auto stereo_image_paths =
      open_maeve::karlsruhe_dataset::getStereoImageFiles(
          data_set_path);
  const auto img = open_maeve::karlsruhe_dataset::getImageMessage(
      camera_header, *stereo_image_paths.left.begin());

  try {
    // Get the camera info
    const auto calib =
        open_maeve::karlsruhe_dataset::calib::createCalib(
            calib_text);

    // Build rosbag
    rosbag::Bag bag;
    bag.open(output_path + "/" + data_set_name +
                 open_maeve::BAG_FILE_EXTENSION,
             rosbag::bagmode::Write);

    auto error_encountered = false;
    auto it_left_image_path = stereo_image_paths.left.begin();
    auto it_right_image_path = stereo_image_paths.right.begin();
    std::string insdata_row_text;
    std::istringstream insdata_stream(insdata_text);
    while (std::getline(insdata_stream, insdata_row_text)) {
      // Check input
      const auto left_error =
          (it_left_image_path == std::end(stereo_image_paths.left));
      const auto right_error =
          (it_right_image_path == std::end(stereo_image_paths.right));
      if (left_error || right_error) {
        std::cerr << "Mismatch in number of left or right camera images and "
                     "insdata rows. Cannot continue.";
        error_encountered = true;
        break;
      }

      // Get messages
      const auto gps_imu_msg = open_maeve::karlsruhe_dataset::
          insdataRow::convertToGPS_IMU(insdata_row_text, global_name, imu_name);

      auto img_header = gps_imu_msg.imu.header;
      img_header.frame_id = camera_name;
      const auto left_camera_msg =
          open_maeve::karlsruhe_dataset::getImageMessage(
              img_header, *it_left_image_path);
      const auto right_camera_msg =
          open_maeve::karlsruhe_dataset::getImageMessage(
              img_header, *it_right_image_path);

      const auto camera_info =
          open_maeve::karlsruhe_dataset::calib::convertToCameraInfo(
              img_header, calib, img->width, img->height);

      // Get transform from world to odom
      const auto world_T_odom =
          open_maeve::getStampedTransformFromPose(
              gps_imu_msg.imu.header, gps_imu_msg.imu.pose.pose, imu_name,
              global_name);

      // Get transform from odom to camera
      const auto odom_T_camera = open_maeve::karlsruhe_dataset::
          getStampedTransformFromCameraToIMU(gps_imu_msg.imu.header, imu_name,
                                             camera_name);

      // Publish tf tree
      // TODO(me): publish static transform for left/right cameras?
      const auto& imu_stamp = gps_imu_msg.imu.header.stamp;
      tf2_msgs::TFMessage tf_msg;
      tf_msg.transforms.push_back(odom_T_camera);
      tf_msg.transforms.push_back(world_T_odom);
      //      bag.write(open_maeve::TF_TOPIC, imu_stamp, tf_msg);

      bag.write("/" + camera_name + "/left/image_raw", img_header.stamp,
                *left_camera_msg);
      bag.write("/" + camera_name + "/left/camera_info",
                camera_info.left.header.stamp, camera_info.left);
      bag.write("/" + camera_name + "/right/image_raw", img_header.stamp,
                *right_camera_msg);
      bag.write("/" + camera_name + "/right/camera_info",
                camera_info.right.header.stamp, camera_info.right);
      bag.write("/" + imu_name + "/odom", gps_imu_msg.imu.header.stamp,
                gps_imu_msg.imu);
      bag.write("/" + imu_name + "/gps", gps_imu_msg.gps.header.stamp,
                gps_imu_msg.gps);

      // Get next round of info
      ++it_left_image_path;
      ++it_right_image_path;
    }

    // Done writing to bag file
    bag.close();

    if (error_encountered) {
      return EXIT_FAILURE;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error encountered building bag file: " << e.what()
              << "\nCannot continue.\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
