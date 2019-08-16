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
#include "feature_flow/params.h"

#include <limits>

namespace open_maeve {

bool FeatureFlowParams::load(const ros::NodeHandle& nh) {
  // Load parameters.
  LOAD_STRUCT_PARAM(ff, max_homographies);
  LOAD_STRUCT_PARAM(ff, min_keypoints);
  LOAD_STRUCT_PARAM(ff, ransac_reprojection_error_threshold);
  LOAD_STRUCT_PARAM(ff, good_match_portion);
  LOAD_STRUCT_PARAM(ff, lsh_table_number);
  LOAD_STRUCT_PARAM(ff, lsh_key_size);
  LOAD_STRUCT_PARAM(ff, lsh_multi_probe_level);
  LOAD_STRUCT_PARAM(ff, threshold_level);
  LOAD_STRUCT_PARAM(ff, octaves);
  LOAD_STRUCT_PARAM(ff, pattern_scales);
  LOAD_PARAM(translation_threshold);
  LOAD_PARAM(scale_threshold);
  LOAD_PARAM(identity_threshold);
  LOAD_PARAM(camera_topic);
  LOAD_PARAM(viz_topic);
  LOAD_PARAM(skip_frames);

  // Sanity check parameters.
  CHECK_GE(scale_threshold, 0.0);
  CHECK_GE(identity_threshold, 0.0);
  CHECK_GE(skip_frames, 0);
  CHECK_STRICTLY_POSITIVE(ff.min_keypoints);
  CHECK_GE(ff.ransac_reprojection_error_threshold, 0);
  CHECK_CONTAINS_CLOSED(ff.good_match_portion, 0.0f, 1.0f);
  CHECK_STRICTLY_POSITIVE(ff.lsh_table_number);
  CHECK_STRICTLY_POSITIVE(ff.lsh_key_size);
  CHECK_STRICTLY_POSITIVE(ff.lsh_multi_probe_level);
  CHECK_NONEMPTY(camera_topic);
  CHECK_GE(ff.threshold_level, 0);
  CHECK_GE(ff.octaves, 0);
  CHECK_STRICTLY_POSITIVE(ff.pattern_scales);

  // Interpret translation threshold.
  if (std::signbit(translation_threshold) ||
      !std::isfinite(translation_threshold)) {
    translation_threshold = std::numeric_limits<double>::quiet_NaN();
  }

  // All (probably) good.
  return true;
}

}  // namespace open_maeve
