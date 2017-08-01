#include "./params.h"

namespace maeve_automation_core {

bool FeatureFlowParams::load(const ros::NodeHandle& nh) {
  // Load parameters.
  LOAD_PARAM(good_match_portion);
  LOAD_PARAM(lsh_table_number);
  LOAD_PARAM(lsh_key_size);
  LOAD_PARAM(lsh_multi_probe_level);
  LOAD_PARAM(threshold_level);
  LOAD_PARAM(octaves);
  LOAD_PARAM(pattern_scales);
  LOAD_PARAM(camera_topic);

  // Sanity check parameters.
  CHECK_CONTAINS_CLOSED(good_match_portion, 0.0f, 1.0f);
  CHECK_STRICTLY_POSITIVE(lsh_table_number);
  CHECK_STRICTLY_POSITIVE(lsh_key_size);
  CHECK_STRICTLY_POSITIVE(lsh_multi_probe_level);
  CHECK_NONEMPTY(camera_topic);
  CHECK_GE(threshold_level, 0);
  CHECK_GE(octaves, 0);
  CHECK_STRICTLY_POSITIVE(pattern_scales);

  // All (probably) good.
  return true;
}

}  // namespace maeve_automation_core
