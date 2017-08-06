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
#include "maeve_automation_core/maeve_expansion_segmentation/connected_component_tracking.h"

#include <algorithm>
#include <limits>

namespace maeve_automation_core {
ConnectedComponentTracker::ConnectedComponentTracker(const Params& params)
    : next_id(0), params_(params), frame_info_buffer_(params_.buffer_size) {}

ConnectedComponentTracker::ContourInfo::ContourInfo()
    : area(std::numeric_limits<double>::quiet_NaN()), ignore(false) {}

size_t ConnectedComponentTracker::getNextId() { return next_id++; }

ConnectedComponentTracker::FrameInfo
ConnectedComponentTracker::computeFrameInfo(const cv::Mat& edges) const {
  FrameInfo frame_info;

  // Copy information.
  edges.copyTo(frame_info.frame);

  // Compute contours.
  std::vector<Contour> contours;
  cv::findContours(frame_info.frame, contours, frame_info.hierarchy,
                   CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  // Compute contour info.
  frame_info.contours.reserve(contours.size());
  std::for_each(contours.begin(), contours.end(), [&](Contour& contour) {
    const auto area = cv::contourArea(contour);
    frame_info.contours.push_back(std::move(ContourInfo()));
    frame_info.contours.back().area = area;
    frame_info.contours.back().contour.swap(contour);
    frame_info.contours.back().ignore = (area < params_.min_component_size) ||
                                        (area > params_.max_component_size);
  });

  // Compute components.
  std::for_each(frame_info.contours.begin(), frame_info.contours.end(),
                [&](ContourInfo& contour) {
                  const auto& f = frame_info.frame;
                  contour.component = cv::Mat::zeros(f.rows, f.cols, f.type());
                  cv::drawContours(contour.component,
                                   std::vector<Contour>(1, contour.contour), -1,
                                   cv::Scalar(255), CV_FILLED);
                });

  // Return.
  return filterFrame(frame_info);
}

ConnectedComponentTracker::FrameInfo ConnectedComponentTracker::filterFrame(
    const FrameInfo& frame_info) const {
  // The filtered frame.
  FrameInfo filtered_frame;

  // Preserve edge information.
  frame_info.frame.copyTo(filtered_frame.frame);

  // Destroy hierarchy information.
  filtered_frame.hierarchy.clear();

  // Perform filtering.
  const auto& h = frame_info.hierarchy;
  for (auto i = 0; i < h.size(); ++i) {
    // Only draw leaves in contour hierarchy?
    auto child_idx = i;
    while (params_.only_leaves && (h[child_idx][2] >= 0)) {
      child_idx = h[child_idx][2];
    }

    // Keep or ignore?
    const auto& contour_info = frame_info.contours[child_idx];
    if (!contour_info.ignore) {
      filtered_frame.contours.push_back(frame_info.contours[child_idx]);
    }
  }

  // Done.
  return filtered_frame;
}

bool ConnectedComponentTracker::addEdgeFrame(const cv::Mat& edges) {
  frame_info_buffer_.push_back(std::move(computeFrameInfo(edges)));
  if (frame_info_buffer_.size() < params_.buffer_size) {
    return false;
  }

  // Store track candidates here.
  std::unordered_map<int, cv::Mat> candidate_tracks;

  // Do tracking:
  // * Find contours
  // * Compute contour components
  // * Add each component that satisfies size bounds to candidate_tracks_
  // * Perform IOU association

  return true;
}
}  // namespace maeve_automation_core
