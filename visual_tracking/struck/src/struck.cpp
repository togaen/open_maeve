// Copyright 2017 Maeve Automation
#include "maeve_automation_core/struck/struck.h"

#include <ros/console.h>

#include <fstream>

namespace maeve_automation_core {

void rectangle(cv::Mat& rMat, const FloatRect& rRect,
               const cv::Scalar& rColour) {
  IntRect r(rRect);
  rectangle(rMat, cv::Point(r.XMin(), r.YMin()), cv::Point(r.XMax(), r.YMax()),
            rColour);
}

}  // namespace maeve_automation_core
