#include "params.h"

bool StruckVisualTrackingParams::load(const ros::NodeHandle& nh) {
  LOAD_PARAM(camera_topic);
  LOAD_PARAM(tracker_image_topic);
  LOAD_PARAM(tracker_bb_topic);
	LOAD_PARAM(enable_viz);
	LOAD_PARAM(quietMode);
	LOAD_PARAM(debugMode);
	LOAD_PARAM(frameWidth);
	LOAD_PARAM(frameHeight);
	LOAD_PARAM(seed);
	LOAD_PARAM(searchRadius);
	LOAD_PARAM(svmC);
	LOAD_PARAM(svmBudgetSize);
	LOAD_PARAM(feature);
	CHECK_NONEMPTY(feature);
	CHECK_NONEMPTY(tracker_bb_topic);

  return SanityCheckConfig(*this);
}

bool StruckVisualTrackingParams::SanityCheckStruckConfig(const Config& c) {
	CHECK_NONEMPTY(c.features);
	return SanityCheckConfig(c);
}

Config StruckVisualTrackingParams::toStruckConfig() const {
  Config config;

	config.quietMode = quietMode;
	config.debugMode = debugMode;
	
	config.sequenceBasePath = "";
	config.sequenceName = "";
	config.resultsPath = "";
	
	config.frameWidth = frameWidth;
	config.frameHeight = frameHeight;
	
	config.seed = seed;
	config.searchRadius = searchRadius;
	config.svmC = svmC;
	config.svmBudgetSize = svmBudgetSize;
	
	config.features.clear();
	std::istringstream iss(feature);
	config.ParseFeatureString(iss);

	return config;
}

