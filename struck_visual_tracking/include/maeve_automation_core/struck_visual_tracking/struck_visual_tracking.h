#pragma once

#include <string>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"

#include "maeve_automation_core/struck_visual_tracking/Tracker.h"
#include "maeve_automation_core/struck_visual_tracking/Config.h"

/// \brief Convenience struct for initializing tracker.
struct TrackerInit {
	TrackerInit() : doInitialise(false), valid(false), useCamera(false), startFrame(-1), endFrame(-1), scaleW(1.f), scaleH(1.f) {}
  bool doInitialise;
	bool valid;
	bool useCamera;
	cv::VideoCapture cap;
	int startFrame;
	int endFrame;
	FloatRect initBB;
	std::string imgFormat;
	float scaleW;
	float scaleH;
};  // struct TrackerInit

bool showOutput(const Config& conf, const cv::Mat& result, TrackerInit& tracker_init, int frameInd, bool paused); 

bool initializeTracker(const Config& conf, TrackerInit& tracker_init, Tracker& tracker, cv::Mat& frame, cv::Mat& result, int frameInd);

TrackerInit BuildTrackerInit(const Config& conf);

void rectangle(cv::Mat& rMat, const FloatRect& rRect, const cv::Scalar& rColour);

bool TrackFromSequence(const std::string& sequence_base_path);
