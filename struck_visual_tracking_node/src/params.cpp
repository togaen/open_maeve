#include "params.h"

#define LOAD_PARAM(var) \
	if (!nh.getParam(#var, var)) {\
		return false;\
	}\
  debug_out << #var << ": " << var << "\n";

bool StruckVisualTrackingParams::load(const ros::NodeHandle& nh) {
  LOAD_PARAM(camera_topic);
  LOAD_PARAM(quietMode);
	LOAD_PARAM(debugMode);
	LOAD_PARAM(sequenceBasePath);
	LOAD_PARAM(resultsPath);
	LOAD_PARAM(sequenceName);
	LOAD_PARAM(frameWidth);
	LOAD_PARAM(frameHeight);
	LOAD_PARAM(seed);
	LOAD_PARAM(searchRadius);
	LOAD_PARAM(svmC);
	LOAD_PARAM(svmBudgetSize);
	LOAD_PARAM(feature);
  return true;
}

std::ostream& operator<<(std::ostream& os, const StruckVisualTrackingParams& params) {
  return os << params.debug_out.str();
}
