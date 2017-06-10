#include "maeve_automation_core/struck_visual_tracking/struck_visual_tracking.h"

#include <fstream>

#include "ros/console.h"

static const int kLiveBoxWidth = 80;
static const int kLiveBoxHeight = 80;

void prepareCameraTrackingFrame(const cv::Mat& frameOrig, const Config& conf, TrackerInit& tracker_init, Tracker& tracker, cv::Mat& frame, cv::Mat& result) {
			resize(frameOrig, frame, cv::Size(conf.frameWidth, conf.frameHeight));
			flip(frame, frame, 1);
			frame.copyTo(result);
			if (tracker_init.doInitialise)
			{
				if (tracker.IsInitialised())
				{
					tracker.Reset();
				}
				else
				{
					tracker.Initialise(frame, tracker_init.initBB);
				}
				tracker_init.doInitialise = false;
			}
			else if (!tracker.IsInitialised())
			{
				rectangle(result, tracker_init.initBB, CV_RGB(255, 255, 255));
			}
}

TrackerInit buildTrackerInit(const Config& conf, const std::string& camera_topic) {
	auto tracker_init = TrackerInit();
	tracker_init.camera_topic = camera_topic;

  // if no sequence specified then use the camera
	tracker_init.useCamera = conf.sequenceName.empty();
	
	if (tracker_init.useCamera)
	{
		tracker_init.startFrame = 0;
		tracker_init.endFrame = INT_MAX;

		tracker_init.initBB = IntRect(conf.frameWidth/2-kLiveBoxWidth/2, conf.frameHeight/2-kLiveBoxHeight/2, kLiveBoxWidth, kLiveBoxHeight);
	}

	// Initialized a valid object.
	tracker_init.valid = true;
	return tracker_init;
}

void rectangle(cv::Mat& rMat, const FloatRect& rRect, const cv::Scalar& rColour)
{
	IntRect r(rRect);
	rectangle(rMat, cv::Point(r.XMin(), r.YMin()), cv::Point(r.XMax(), r.YMax()), rColour);
}
