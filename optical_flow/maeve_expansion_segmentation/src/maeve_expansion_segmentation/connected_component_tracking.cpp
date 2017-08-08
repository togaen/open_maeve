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
#include <utility>
#include <vector>

namespace maeve_automation_core {
ConnectedComponentTracker::ConnectedComponentTracker(const Params& params)
    : next_color_(0),
      next_id_(0),
      params_(params),
      frame_info_buffer_(params_.buffer_size) {}

ConnectedComponentTracker::ContourInfo::ContourInfo()
    : area(std::numeric_limits<double>::quiet_NaN()),
      ignore(false),
      consumed(false) {}

ConnectedComponentTracker::Track::Track()
    : current(false), timestamp(-std::numeric_limits<double>::infinity()) {}

ConnectedComponentTracker::FrameInfo::FrameInfo(const double ts)
    : timestamp(ts) {}

ConnectedComponentTracker::Id ConnectedComponentTracker::getNextId() {
  return next_id_++;
}

int ConnectedComponentTracker::getNextColor() { return next_color_++; }

const boost::circular_buffer<ConnectedComponentTracker::FrameInfo>&
ConnectedComponentTracker::getFrameInfoBuffer() const {
  return frame_info_buffer_;
}

const ConnectedComponentTracker::Tracks& ConnectedComponentTracker::getTracks()
    const {
  return tracks_;
}

double ConnectedComponentTracker::IOU(const cv::Mat& binary_image1,
                                      const cv::Mat& binary_image2) {
  // Initialize containers.
  cv::Mat set_intersection = cv::Mat::zeros(
      binary_image1.rows, binary_image1.cols, binary_image1.type());
  cv::Mat set_union = cv::Mat::zeros(binary_image1.rows, binary_image1.cols,
                                     binary_image1.type());

  // Compute sets.
  cv::bitwise_and(binary_image1, binary_image2, set_intersection);
  cv::bitwise_or(binary_image1, binary_image2, set_union);

  // Compute area.
  const auto intersection_area =
      static_cast<double>(cv::countNonZero(set_intersection));
  const auto union_area = static_cast<double>(cv::countNonZero(set_union));

  // Compute IOU and done; operation undefined when union is empty.
  return (union_area == 0.0) ? std::numeric_limits<double>::quiet_NaN()
                             : (intersection_area / union_area);
}

void ConnectedComponentTracker::updateTrack(const double timestamp,
                                            Track& track,
                                            ContourInfo& contour_info) {
  std::swap(track.contour_info, contour_info);
  contour_info.consumed = true;
  track.timestamp = timestamp;
}

ConnectedComponentTracker::Id
ConnectedComponentTracker::associateContourToTrack(
    const ContourInfo& contour_info) const {
  auto max_id = INVALID_ID;
  auto max_iou = -std::numeric_limits<double>::infinity();
  std::for_each(std::begin(tracks_), std::end(tracks_),
                [&](const Tracks::value_type& pair) {
                  // Compute IOU.
                  const auto iou = ConnectedComponentTracker::IOU(
                      pair.second.contour_info.component,
                      contour_info.component);

                  // Track 'best' match.
                  if (iou >= max_iou) {
                    max_iou = iou;
                    max_id = pair.first;
                  }
                });

  // Return the found id iff the IOU is above the threshold.
  return (max_iou > params_.IOU_threshold) ? max_id : INVALID_ID;
}

ConnectedComponentTracker::FrameInfo
ConnectedComponentTracker::computeFrameInfo(const double timestamp,
                                            const cv::Mat& edges) const {
  FrameInfo frame_info(timestamp);

  // Copy information.
  edges.copyTo(frame_info.frame);

  // Compute contours.
  std::vector<Contour> contours;
  cv::findContours(frame_info.frame, contours, frame_info.hierarchy,
                   CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  // Compute contour info.
  frame_info.contours.reserve(contours.size());
  std::for_each(std::begin(contours), std::end(contours),
                [&](Contour& contour) {
                  const auto area = cv::contourArea(contour);
                  frame_info.contours.push_back(std::move(ContourInfo()));
                  frame_info.contours.back().area = area;
                  frame_info.contours.back().contour.swap(contour);
                  frame_info.contours.back().ignore =
                      (area < params_.min_component_size) ||
                      (area > params_.max_component_size);
                });

  // Compute components.
  std::for_each(std::begin(frame_info.contours), std::end(frame_info.contours),
                [&](ContourInfo& contour) {
                  const auto& f = frame_info.frame;
                  contour.component = cv::Mat::zeros(f.rows, f.cols, f.type());
                  cv::drawContours(contour.component,
                                   std::vector<Contour>(1, contour.contour), -1,
                                   cv::Scalar(255), CV_FILLED);
                });

  // Return.
  return frame_info;
}

ConnectedComponentTracker::FrameInfo ConnectedComponentTracker::filterFrame(
    const FrameInfo& frame_info) const {
  // The filtered frame.
  FrameInfo filtered_frame(frame_info.timestamp);

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

bool ConnectedComponentTracker::addEdgeFrame(const double timestamp,
                                             const cv::Mat& edges) {
  frame_info_buffer_.push_back(std::move(computeFrameInfo(timestamp, edges)));
  if (frame_info_buffer_.size() < params_.buffer_size) {
    return false;
  }

  // Apply filters.
  auto filtered_frame = filterFrame(frame_info_buffer_.back());

  // Set all tracks to need updating.
  std::for_each(std::begin(tracks_), std::end(tracks_),
                [](Tracks::value_type& pair) { pair.second.current = false; });

  // Perform association.
  std::for_each(std::begin(filtered_frame.contours),
                std::end(filtered_frame.contours),
                [&](ContourInfo& contour_info) {
                  const auto id = associateContourToTrack(contour_info);
                  if (id != INVALID_ID) {
                    ConnectedComponentTracker::updateTrack(
                        filtered_frame.timestamp, tracks_[id], contour_info);
                  }
                });

  // Prune expired tracks.
  const auto pre_size = tracks_.size();
  for (auto it = std::begin(tracks_); it != std::end(tracks_);) {
    const auto age = (filtered_frame.timestamp - it->second.timestamp);
    const auto carousel = (age >= params_.max_age);
    if (carousel) {
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }

  // Add new tracks.
  std::for_each(std::begin(filtered_frame.contours),
                std::end(filtered_frame.contours),
                [&](ContourInfo& contour_info) {
                  if (contour_info.consumed) {
                    return;
                  }
                  const auto id = getNextId();
                  tracks_[id].contour_info = std::move(contour_info);
                  tracks_[id].color = getNextColor();
                });

  return true;
}
}  // namespace maeve_automation_core
