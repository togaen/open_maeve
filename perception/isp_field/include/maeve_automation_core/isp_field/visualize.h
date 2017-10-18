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

namespace maeve_automation_core {
/**
 * @brief Compute a visualization of a control horizon.
 *
 * The control horizon is a 1xN matrix, so the visualization stretches the
 * horizon vertically by 'horizon_viz_height' pixels and paints it,
 * vertically centered, onto a black canvas that is of size
 * 'window_viz_height'xN.
 *
 * @param control_horizon The control horizon that is begin visualized.
 * @param horizon_viz_height The visualization height of the control horizon.
 * @param window_viz_height  The height of the visualization window.
 *
 * @return The visualization image, a monochrome image.
 */
cv::Mat computeControlHorizonVisualization(const cv::Mat& control_horizon,
                                           const int horizon_viz_height,
                                           const int window_viz_height);

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
 */
cv::Mat computeISPFieldVisualization(const cv::Mat& isp,
                                     const double lower_bound,
                                     const double upper_bound);
}  // namespace maeve_automation_core
