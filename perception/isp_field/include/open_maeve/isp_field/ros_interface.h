/*
 * Copyright 2017 Maeve Automation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#pragma once

#include <ros/ros.h>

#include "open_maeve/isp_field/shape_parameters.h"

namespace open_maeve {
/**
 * @brief Convenience method for filling a parameter object from the ROS
 * parameter server.
 *
 * @note To use this function, the parameters must exist in a node-relative
 * parameter namespace called 'isp_controller_params'.
 *
 * @param nh ROS node handle.
 * @param ns The node-relative namespace that the parameters live in.
 * @param isp_controller_params The parameter object to fill out.
 *
 * @return True if 'isp_controller_params' was successfully filled out;
 * otherwise false.
 */
bool loadShapeParamsROS_Params(const ros::NodeHandle& nh, const std::string& ns,
                               ShapeParameters& shape_params);
}  // namespace open_maeve
