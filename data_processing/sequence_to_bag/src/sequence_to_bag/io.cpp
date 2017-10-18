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
#include "sequence_to_bag/io.h"

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <yaml-cpp/yaml.h>
#include <boost/range/iterator_range.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdlib>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace {
const auto CLOCK_HZ_DEFAULT = 1000;
}  // namespace

namespace maeve_automation_core {
std::ostream& operator<<(std::ostream& os, const MetaInfo& meta_info) {
  os << meta_info.name << "\n";
  os << meta_info.description << "\n";
  os << meta_info.raw_image_dir << "\n";
  os << meta_info.seg_image_dir << "\n";
  os << meta_info.fps;
  return os;
}

boost::optional<std::tuple<std::map<int, sensor_msgs::ImagePtr>,
                           std::map<int, sensor_msgs::ImagePtr>>>
getSortedIndexedImageLists(const std::string& raw_image_dir,
                           const std::string& seg_image_dir) {
  const auto raw_files = maeve_automation_core::getFileList(raw_image_dir);
  const auto raw_files_idx =
      maeve_automation_core::getSortedIndexedFileList(raw_files);
  const auto raw_images_idx =
      maeve_automation_core::getSortedIndexedImages(raw_files_idx);

  const auto seg_files = maeve_automation_core::getFileList(seg_image_dir);
  const auto seg_files_idx =
      maeve_automation_core::getSortedIndexedFileList(seg_files);
  const auto seg_images_idx =
      maeve_automation_core::getSortedIndexedImages(seg_files_idx);

  if (raw_files_idx.size() != seg_images_idx.size()) {
    return boost::none;
  }

  return std::make_tuple(std::move(raw_images_idx), std::move(seg_images_idx));
}

double getClockHz(const std::string& str) {
  char* pEnd = nullptr;
  const auto hz = std::strtod(str.c_str(), &pEnd);
  if (str.empty() || (*pEnd != '\0')) {
    return CLOCK_HZ_DEFAULT;
  }
  return hz;
}

std::string constructMetaYamlPath(const std::string& dir) {
  return dir + "/meta.yaml";
}

boost::optional<MetaInfo> getMetaInfo(const std::string& path) {
  // Read meta info from data set.
  YAML::Node config;
  const auto meta_path = constructMetaYamlPath(path);
  try {
    config = YAML::LoadFile(meta_path);
  } catch (...) {
    return boost::none;
  }

  // \TODO(me) Maybe check that the keys exist.

  // Get meta info.
  MetaInfo meta_info;
  meta_info.name = config["sequence_meta"]["name"].as<std::string>();
  meta_info.description =
      config["sequence_meta"]["description"].as<std::string>();
  meta_info.raw_image_dir =
      path + "/" + config["sequence_meta"]["raw"].as<std::string>();
  meta_info.seg_image_dir =
      path + "/" + config["sequence_meta"]["segmented"].as<std::string>();
  meta_info.fps = config["sequence_meta"]["fps"].as<double>();

  return meta_info;
}

std::map<int, sensor_msgs::ImagePtr> getSortedIndexedImages(
    const std::map<int, std::string>& file_list) {
  std::map<int, sensor_msgs::ImagePtr> image_list;

  std::for_each(
      std::begin(file_list), std::end(file_list),
      [&](const std::map<int, std::string>::value_type& p) {
        cv::Mat img = cv::imread(p.second);
        auto msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        image_list[p.first] = msg;
      });

  return image_list;
}

std::map<int, std::string> getSortedIndexedFileList(
    const std::vector<boost::filesystem::path>& file_list) {
  std::map<int, std::string> indexed_file_list;

  std::for_each(std::begin(file_list), std::end(file_list),
                [&](const boost::filesystem::path& f) {
                  // String version of file basename.
                  const auto s = f.stem().string();

                  // Get the first index of the last integer portion.
                  auto found = false;
                  auto i = static_cast<int>(s.length()) - 1;
                  while ((i >= 0) && std::isdigit(s[i])) {
                    found = true;
                    --i;
                  }

                  // No integer portion; skip it.
                  if (!found) {
                    return;
                  }

                  // Extract index and convert; add 1 to index because it points
                  // to first character before integer sequence.
                  const auto str_idx = s.substr(i + 1);
                  const auto int_idx = std::atoi(str_idx.c_str());

                  // Sort and add fully-qualified path to list.
                  indexed_file_list[int_idx] = f.string();
                });

  return indexed_file_list;
}

std::vector<boost::filesystem::path> getFileList(const std::string& path) {
  std::vector<boost::filesystem::path> file_list;

  if (boost::filesystem::is_directory(path)) {
    for (auto& entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(path), {})) {
      const auto f = entry.path().filename().string();
      if (boost::filesystem::is_directory(entry) || f.empty() ||
          (f[0] == '.')) {
        continue;
      }
      file_list.push_back(entry.path());
    }
  }

  return file_list;
}
}  // namespace maeve_automation_core
