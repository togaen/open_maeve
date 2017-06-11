#pragma once

#include <string>
#include <fstream>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"

#include "maeve_automation_core/struck_visual_tracking/Tracker.h"
#include "maeve_automation_core/struck_visual_tracking/Config.h"

void rectangle(cv::Mat& rMat, const FloatRect& rRect, const cv::Scalar& rColour);
