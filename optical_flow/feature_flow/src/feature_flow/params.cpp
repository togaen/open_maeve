#include "./params.h"

namespace maeve_automation_core {

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
  LOAD_PARAM(camera_topic);

  // Sanity check parameters.
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

  // All (probably) good.
  return true;
}

}  // namespace maeve_automation_core
