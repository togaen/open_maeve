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

namespace {
namespace po = boost::program_options;
}  // namespace

int main(int argc, char** argv) {
  boost::optional<std::string> data_set_path_opt;
  boost::optional<std::string> output_path_opt;
  constexpr auto CAMERA_DEFAULT = "camera";

  po::options_description desc(
      "Karlsruhe Dataset sequencer. See README.md for details.\nAvailable "
      "options are listed below. Arguments without default "
      "values are required",
      maeve_automation_core::PROGRAM_OPTIONS_LINE_LENGTH);
  desc.add_options()("help,h", "Print help and exit.")(
      "data-set-path,d", po::value(&data_set_path_opt)->required(),
      "Absolute path to the data set.")(
      "bag-output-dir,o", po::value(&output_path_opt)->required(),
      "Absolute path to the directory for the output bag file.")(
      "camera-name,c", po::value<std::string>()->default_value(CAMERA_DEFAULT),
      "Camera name to use for the stereo image stream.");

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
  const auto output_path = *output_path_opt;
  const auto camera_name = vm["camera-name"].as<std::string>();

  return EXIT_SUCCESS;
}
