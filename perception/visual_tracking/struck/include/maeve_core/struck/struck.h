/*
 * Copyright 2017 Maeve Automation
 *
 * Struck: Structured Output Tracking with Kernels
 *
 * Derived work of code to accompany the paper:
 *   Struck: Structured Output Tracking with Kernels
 *   Sam Hare, Amir Saffari, Philip H. S. Torr
 *   International Conference on Computer Vision (ICCV), 2011
 *
 * Copyright (C) 2011 Sam Hare, Oxford Brookes University, Oxford, UK
 *
 * This file is a derivative work of Struck.
 *
 * Struck is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Struck is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Struck.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include <fstream>
#include <string>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"

#include "maeve_core/struck/Config.h"
#include "maeve_core/struck/Tracker.h"

namespace maeve_core {

/**
 * @brief Draw a rectangle described by rRect, with rColour, onto rMat.
 *
 * @param rMat The output image.
 * @param rRect The rectangle geometry.
 * @param rColour The rectangle color.
 */
void rectangle(cv::Mat& rMat, const FloatRect& rRect,
               const cv::Scalar& rColour);

}  // namespace maeve_core
