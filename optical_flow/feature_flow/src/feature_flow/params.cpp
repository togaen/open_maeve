#include "./params.h"

namespace maeve_automation_core {

bool FeatureFlowParams::load(const ros::NodeHandle& nh) {
  LOAD_PARAM(threshold_level);
  LOAD_PARAM(octaves);
  LOAD_PARAM(pattern_scales);
  LOAD_PARAM(camera_topic);
  CHECK_NONEMPTY(camera_topic);
  CHECK_GE(threshold_level, 0);
  CHECK_GE(octaves, 0);
  CHECK_CONTAINS_CLOSED(pattern_scales, 0.0, 1.0);

  return true;
}

}  // namespace maeve_automation_core
