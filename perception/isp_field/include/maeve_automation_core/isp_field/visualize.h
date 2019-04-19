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

#include <opencv2/opencv.hpp>

#include "maeve_automation_core/maeve_geometry/interval.h"

namespace maeve_automation_core {
/**
 * @brief Compute a visualization of a horizon.
 *
 * @pre horizon_viz_height and window_viz_height are both > 0; 'channel'
 * specifies a valid channel in 'horizon'.
 *
 * The horizon is a 1xN matrix, so the visualization stretches the
 * horizon vertically by 'horizon_viz_height' pixels and paints it,
 * vertically centered, onto a black canvas that is of size
 * 'window_viz_height'xN.
 *
 * @note If horizon_viz_height is greater than window_viz_height,
 * horizon_viz_height is resized to window_viz_height.
 *
 * @param horizon The horizon that is begin visualized.
 * @param channel Which channel in the horizon image to visualize.
 * @param horizon_viz_height The visualization height of the control horizon.
 * @param window_viz_height  The height of the visualization window.
 * @param lower_bound The lower saturation bound.
 * @param upper_bound The upper saturation bound.
 *
 * @return The visualization image, a monochrome image.
 */
cv::Mat computeHorizonVisualization(const cv::Mat& horizon, const int channel,
                                    const int horizon_viz_height,
                                    const int window_viz_height,
                                    const Interval_d& bounds);

/**
 * @brief Compute a visualization of an Image Space Potential field.
 *
 * @pre The input ISP field is negatively affinely extended.
 *
 * This function maps repulsive forces to the red channel and attractive forces
 * to the blue channel. The parameters 'lower_bound' and 'upper_bound' specify
 * real-valued ranges [lower_bound, 0] and [0, upper_bound] that map attractive
 * and repulsive potential values, respectively, onto [0, 255] for visualizing.
 * Potential values outside [lower_bound, upper_bound] are saturated at 255. To
 * be meaningful, the bounds must satisfy lower_bound < 0 < upper_bound.
 *
 * @param isp The image space potential field.
 * @param lower_bound The lower saturation bound.
 * @param upper_bound The upper saturation bound.
 *
 * @return An image that visualizes repulsive potentials as red and attractive
 * potentials as blue, with color intensity proportional to potential magnitude.
 * The image type is CV_8UC3.
 */
cv::Mat computeISPFieldVisualization(const cv::Mat& isp,
                                     const Interval_d& bounds);
}  // namespace maeve_automation_core
