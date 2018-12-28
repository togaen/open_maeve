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
#pragma once

#include "sequence_to_bag/sequence_to_bag.h"

#include <array>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace maeve_automation_core {
namespace parallel_domain {

/**
 * @brief Data structure to store information from meta.yaml.
 */
struct MetaInfo {
  /** @brief The name of the data set. */
  std::string name;
  /** @brief The description of the data set. */
  std::string description;
  /** @brief The full path of the raw image portion of the data set. */
  std::string raw_image_dir;
  /** @brief The full path of the segmented image portion of the data set. */
  std::string seg_image_dir;
  /** @brief The framerate the data were captured at. */
  double fps;
};  // struct MetaInfo

/**
 * @brief Stream overload for meta info struct.
 *
 * @param os The stream.
 * @param meta_info The meta info object.
 *
 * @return The stream with meta info added.
 */
std::ostream& operator<<(std::ostream& os, const MetaInfo& meta_info);

/**
 * @brief Convenience function to retrieve all data at once.
 *
 * @param raw_image_dir The full path to the directory containing camera images.
 * @param seg_image_dir The full path to the directory containing segmentations.
 *
 * @return A pair of sorted, indexed image sets.
 */
boost::optional<std::tuple<std::map<int, sensor_msgs::ImagePtr>,
                           std::map<int, sensor_msgs::ImagePtr>>>
getSortedIndexedImageLists(const std::string& raw_image_dir,
                           const std::string& seg_image_dir);

/**
 * @brief Convert clock publish frequency from string to double.
 *
 * @param str The string parameter for clock publish frequency.
 *
 * @return The double version of 'str', or the internally defined default value
 * if 'str' cannot be converted to double.
 */
double getClockHz(const std::string& str);

/**
 * @brief For a given path, construct the file path to the meta information.
 *
 * @param dir The full directory path.
 *
 * @return The full path to the meta information file.
 */
std::string constructMetaYamlPath(const std::string& dir);

/**
 * @brief Read in the meta.yaml file and return a corresponding struct.
 *
 * @param path The fully path to the meta.yaml file.
 *
 * @return A nullable object that either contains the MetaInfo object, or null
 * if an error was encountered.
 */
boost::optional<MetaInfo> getMetaInfo(const std::string& path);

/**
 * @brief Load a set of images given a set of file names.
 *
 * @param file_list The list of filenames.
 *
 * @return A sorted, indexed list of image messages with the same indexing as
 * the given file list.
 */
std::map<int, sensor_msgs::ImagePtr> getSortedIndexedImages(
    const std::map<int, std::string>& file_list);

/**
 * @brief For a given list of fully qualifed filenames, extract integer portion
 * of each name and index the filename by it.
 *
 * @note Filenames that have no integer portion are ignored.
 *
 * @param file_list The list of fully-qualified filenames.
 *
 * @return A map of integer -> filename.
 */
std::map<int, std::string> getSortedIndexedFileList(
    const std::vector<boost::filesystem::path>& file_list);

}  // namespace parallel_domain
}  // namespace maeve_automation_core
