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
  constexpr auto HELP = "help";
  boost::optional<std::string> data_set_path_opt;
  boost::optional<std::string> output_path_opt;
  boost::optional<std::string> camera_name_opt;

  po::options_description desc(
      "Available arguments. All required arguments must be set");
  desc.add_options()(HELP, "Print help and exit.")(
      "data-set-path", po::value(&data_set_path_opt),
      "[Required]: Absolute path to the data set.")(
      "bag-output-dir", po::value(&output_path_opt),
      "Absolute path to the directory that will contain the output bag file.")(
      "camera-name", po::value(&camera_name_opt),
      "[Required]: Camera name to use for the stereo image stream.");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  const auto help_requested = vm.count(HELP);
  const auto required_arg_not_set =
      !(data_set_path_opt && output_path_opt && camera_name_opt);
  if (help_requested || required_arg_not_set) {
    std::cout << desc << "\n";
    return EXIT_SUCCESS;
  }

  const auto data_set_path = *data_set_path_opt;
  const auto output_path = *output_path_opt;
  const auto camera_name = *camera_name_opt;

  return EXIT_SUCCESS;
}
