#include "maeve_automation_core/struck_visual_tracking/struck_visual_tracking.h"

#include <fstream>

#include "ros/console.h"

void rectangle(cv::Mat& rMat, const FloatRect& rRect, const cv::Scalar& rColour)
{
	IntRect r(rRect);
	rectangle(rMat, cv::Point(r.XMin(), r.YMin()), cv::Point(r.XMax(), r.YMax()), rColour);
}
