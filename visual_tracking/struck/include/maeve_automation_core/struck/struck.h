// Copyright 2017 Maeve Automation
#pragma once

#include <fstream>
#include <string>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"

#include "maeve_automation_core/struck/Config.h"
#include "maeve_automation_core/struck/Tracker.h"

namespace maeve_automation_core {

void rectangle(cv::Mat& rMat, const FloatRect& rRect,
               const cv::Scalar& rColour);

}  // namespace maeve_automation_core
