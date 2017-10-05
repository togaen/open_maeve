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

#include <sensor_msgs/Image.h>
#include <boost/filesystem.hpp>

#include <map>
#include <string>
#include <vector>

namespace maeve_automation_core {
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
*@brief For a given list of fully qualifed filenames, extract integer portion
*of each name and index the filename by it.
*
*@note Filenames that have no integer portion are ignored.
*
*@param file_list The list of fully-qualified filenames.
*
*@return A map of integer -> filename.
*/
std::map<int, std::string> getSortedIndexedFileList(
    const std::vector<boost::filesystem::path>& file_list);

/**
 * @brief Get a list of all non-hidden files in the given path.
 *
 * @param path The path of the directory to list files for.
 *
 * @return A vector of fully qualifed filenames. If 'path' is not a valid
 * directory the returned vector will be empty.
 */
std::vector<boost::filesystem::path> getFileList(const std::string& path);
}  // namespace maeve_automation_core
