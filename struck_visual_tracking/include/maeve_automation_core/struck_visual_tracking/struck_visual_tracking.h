#pragma once

#include <string>
#include <fstream>

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
	int startFrame;
	int endFrame;
	FloatRect initBB;
	std::string imgFormat;
	std::string camera_topic;
	float scaleW;
	float scaleH;
};  // struct TrackerInit

bool runTracker(const Config& conf, TrackerInit& tracker_init, Tracker& tracker);

void prepareCameraTrackingFrame(const cv::Mat& frameOrig, const Config& conf, TrackerInit& tracker_init, Tracker& tracker, cv::Mat& frame, cv::Mat& result);

TrackerInit buildTrackerInit(const Config& conf, const std::string& camera_topic);

void rectangle(cv::Mat& rMat, const FloatRect& rRect, const cv::Scalar& rColour);
