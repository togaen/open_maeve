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
#include "maeve_expansion_segmentation/params.h"

namespace maeve_automation_core {

bool MaeveExpansionSegmentationParams::load(const ros::NodeHandle& nh) {
  // Load parameters.
  LOAD_PARAM(camera_topic);
  LOAD_PARAM(enable_viz);
  LOAD_PARAM(viz_te_topic);
  LOAD_PARAM(viz_se_topic);
  LOAD_NS_PARAM(spatial_params, min);
  LOAD_NS_PARAM(spatial_params, max);
  LOAD_NS_PARAM(spatial_params, aperture);
  LOAD_NS_PARAM(temporal_params, history);
  LOAD_NS_PARAM(temporal_params, threshold);
  LOAD_NS_PARAM(temporal_params, shadows);
  LOAD_NS_PARAM(morpho_params, element_type);
  LOAD_NS_PARAM(morpho_params, window_width);
  LOAD_NS_PARAM(morpho_params, window_height);

  // Sanity check parameters.
  CHECK_STRICTLY_POSITIVE(morpho_params.window_width);
  CHECK_ODD(morpho_params.window_width);
  CHECK_STRICTLY_POSITIVE(morpho_params.window_height);
  CHECK_ODD(morpho_params.window_height);
  CHECK_CONTAINS_CLOSED(morpho_params.element_type, -1, 1);
  CHECK_STRICTLY_POSITIVE(temporal_params.history);
  CHECK_STRICTLY_POSITIVE(temporal_params.threshold);
  CHECK_STRICTLY_POSITIVE(spatial_params.aperture);
  CHECK_ODD(spatial_params.aperture);
  CHECK_CONTAINS_CLOSED(spatial_params.min, 0, 255);
  CHECK_CONTAINS_CLOSED(spatial_params.max, 0, 255);
  CHECK_GE(temporal_params.history, 0);
  CHECK_GE(temporal_params.threshold, 0);
  CHECK_LE(spatial_params.min, spatial_params.max);
  CHECK_NONEMPTY(camera_topic);
  if (enable_viz) {
    CHECK_NONEMPTY(viz_te_topic);
    CHECK_NONEMPTY(viz_se_topic);
  }

  // All (probably) good.
  return true;
}

}  // namespace maeve_automation_core
