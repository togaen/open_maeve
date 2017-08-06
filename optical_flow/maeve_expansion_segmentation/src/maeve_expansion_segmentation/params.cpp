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
// Aribtrarily chosen bad value.
static const int BAD_INT = -9;

MaeveExpansionSegmentationParams::SpatialParams::SpatialParams()
    : edge_min(BAD_INT),
      edge_max(BAD_INT),
      edge_aperture(BAD_INT),
      blur_aperture(BAD_INT) {}

bool MaeveExpansionSegmentationParams::SpatialParams::valid() const {
  CHECK_NE(edge_min, BAD_INT);
  CHECK_NE(edge_max, BAD_INT);
  CHECK_NE(edge_aperture, BAD_INT);
  CHECK_NE(blur_aperture, BAD_INT);
  CHECK_GE(edge_aperture, 3);
  CHECK_ODD(edge_aperture);
  if (blur_aperture >= 0) {
    CHECK_ODD(blur_aperture);
    CHECK_GE(blur_aperture, 3);
  }
  CHECK_CONTAINS_CLOSED(edge_min, 0, 255);
  CHECK_CONTAINS_CLOSED(edge_max, 0, 255);
  CHECK_LE(edge_min, edge_max);
  return true;
}

bool MaeveExpansionSegmentationParams::TemporalParams::valid() const {
  CHECK_NE(history, BAD_INT);
  CHECK_NE(threshold, BAD_INT);
  CHECK_STRICTLY_POSITIVE(threshold);
  return true;
}

bool MaeveExpansionSegmentationParams::MorphologicalParams::valid() const {
  CHECK_NE(window_width, BAD_INT);
  CHECK_NE(window_height, BAD_INT);
  CHECK_NE(element_type, BAD_INT);

  CHECK_GE(window_width, 3);
  CHECK_ODD(window_width);
  CHECK_GE(window_height, 3);
  CHECK_ODD(window_height);
  CHECK_CONTAINS_CLOSED(element_type, 0, 1);

  return true;
}

MaeveExpansionSegmentationParams::TemporalParams::TemporalParams()
    : history(BAD_INT), threshold(BAD_INT) {}

MaeveExpansionSegmentationParams::MorphologicalParams::MorphologicalParams()
    : element_type(BAD_INT), window_width(BAD_INT), window_height(BAD_INT) {}

bool MaeveExpansionSegmentationParams::load(const ros::NodeHandle& nh) {
  // Load parameters.
  LOAD_PARAM(camera_topic);
  LOAD_PARAM(enable_viz);
  LOAD_PARAM(viz_te_topic);
  LOAD_PARAM(viz_se_topic);
  LOAD_PARAM(viz_AND_topic);
  LOAD_PARAM(morpho_operations);
  LOAD_NS_PARAM(spatial_params, edge_min);
  LOAD_NS_PARAM(spatial_params, edge_max);
  LOAD_NS_PARAM(spatial_params, edge_aperture);
  LOAD_NS_PARAM(spatial_params, blur_aperture);
  LOAD_NS_PARAM(temporal_params, history);
  LOAD_NS_PARAM(temporal_params, threshold);
  LOAD_NS_PARAM(temporal_params, shadows);
  LOAD_NS_PARAM(dilation_params, element_type);
  LOAD_NS_PARAM(dilation_params, window_width);
  LOAD_NS_PARAM(dilation_params, window_height);
  LOAD_NS_PARAM(erosion_params, element_type);
  LOAD_NS_PARAM(erosion_params, window_width);
  LOAD_NS_PARAM(erosion_params, window_height);

  // Sanity check parameters.
  for (const auto& op : morpho_operations) {
    CHECK_CONTAINS_CLOSED(op, 0, 1);
  }
  CHECK_NONEMPTY(camera_topic);
  if (enable_viz) {
    CHECK_NONEMPTY(viz_te_topic);
    CHECK_NONEMPTY(viz_se_topic);
  }

  // All (probably) good.
  return true;
}

}  // namespace maeve_automation_core
