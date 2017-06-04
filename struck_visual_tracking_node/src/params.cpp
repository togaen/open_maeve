#include "params.h"

#include <sstream>

#define LOAD_PARAM(var) \
	if (!nh.getParam(#var, var)) {\
		return false;\
	}\
  loaded_param_set << #var << ": " << var << "\n";

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
	LOAD_PARAM(features);
  return true;
}

Config StruckVisualTrackingParams::toStruckConfig() const {
  Config config;

	config.quietMode = quietMode;
	config.debugMode = debugMode;
	
	config.sequenceBasePath = sequenceBasePath;
	config.sequenceName = sequenceName;
	config.resultsPath = resultsPath;
	
	config.frameWidth = frameWidth;
	config.frameHeight = frameHeight;
	
	config.seed = seed;
	config.searchRadius = searchRadius;
	config.svmC = svmC;
	config.svmBudgetSize = svmBudgetSize;
	
	config.features.clear();
	std::istringstream iss(features);
	config.ParseFeatureString(iss);

	return config;
}

std::ostream& operator<<(std::ostream& os, const StruckVisualTrackingParams& params) {
  return os << params.loaded_param_set.str();
}
