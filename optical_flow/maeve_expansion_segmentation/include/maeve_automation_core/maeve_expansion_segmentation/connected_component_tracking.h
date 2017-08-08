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

#include <boost/circular_buffer.hpp>
#include <opencv2/opencv.hpp>

#include <unordered_map>
#include <vector>

namespace maeve_automation_core {

/**
 * @brief Tracker class to perform simple connected component tracking.
 */
class ConnectedComponentTracker {
 public:
  /** @name Convenience typedefs */
  /*@{
   */
  typedef int Id;
  typedef std::vector<cv::Point> Contour;
  typedef std::vector<cv::Vec4i> ContourHierarchy;
  struct Track;
  typedef std::unordered_map<Id, Track> Tracks;
  /** @} */

  /**
   * @brief Parameter for the tracker.
   */
  struct Params {
    /** @brief Only consider leaves in the connected component hierarchy. */
    bool only_leaves;
    /** @brief Size of the circular buffer used to store image frames. */
    int buffer_size;
    /** @brief Minimum size for a connected component to be considered. */
    int min_component_size;
    /** @brief Maximum size for a connected component to be considered. */
    int max_component_size;
    /** @brief IOU threshold for determining temporal association. */
    double IOU_threshold;
    /** @brief Maximum length of time before a track is killed. */
    double max_age;
  };  // struct Params

  /**
   * @brief Store an image contour and associated information
   */
  struct ContourInfo {
    /** @brief The OpenCV contour. */
    Contour contour;
    /** @brief The connected component drawn in an image. */
    cv::Mat component;
    /** @brief The computed pixel area of the component. */
    double area;
    /** @brief Whether to ignore this contour during tracking. */
    bool ignore;
    /** @brief Whether this contour object has been consumed by the tracker. */
    bool consumed;
    /**
     * @brief Constructor: initialize ignore/consumed to false and area to NaN.
     */
    ContourInfo();
  };  // struct ContourInfo

  /**
   * @brief Store track information.
   */
  struct Track {
    /** @brief Single color index [r, g, b] -> [0, 1, 2] */
    int color;
    /** @brief The contour information associated with this track. */
    ContourInfo contour_info;
    /** @brief Whether this track is 'current' (i.e., has been updated). */
    bool current;
    /** @brief Time of the last measurement update for this track. */
    double timestamp;
    /**
     * @brief Constructor: initialize current to false and timestamp to -inf.
     */
    Track();
  };  // struct Track

  /**
   * @brief Store information about a given edge detection frame.
   */
  struct FrameInfo {
    /** @brief The detected edges. */
    cv::Mat frame;
    /** @brief The detected contours. */
    std::vector<ContourInfo> contours;
    /** @brief The contour hierarchy. */
    ContourHierarchy hierarchy;
    /** @brief Measurement time of this frame. */
    double timestamp;
    /**
     * @brief Constructor: Initialize timestamp.
     *
     * @param ts The measurement timestamp for this frame.
     */
    explicit FrameInfo(const double ts);
  };  // struct FrameInfo

  /**
   * @brief Constructor: initialize the parameter object.
   *
   * @param params The tracker parameters.
   */
  explicit ConnectedComponentTracker(const Params& params);

  /**
   * @brief Add a frame to the frame buffer; compute associated information.
   *
   * @param timestamp The absolute timestamp of the frame.
   * @param edges The edge detection.
   *
   * @return True if frame buffer has reached size; otherwise false.
   */
  bool addEdgeFrame(const double timestamp, const cv::Mat& edges);

  /**
   * @brief Get a const reference to the frame info buffer.
   *
   * @return A const ref to the frame info buffer.
   */
  const boost::circular_buffer<FrameInfo>& getFrameInfoBuffer() const;

  /**
   * @brief Get a const reference to the set of tracks.
   *
   * @return A const ref to the set of tracks.
   */
  const Tracks& getTracks() const;

 private:
  /**
   * @brief Container to cache intermediate results from IOU operation.
   */
  struct IOUVal {
    /** @brief The binary image representing the intersection. */
    cv::Mat intersection_set;
    /** @brief The binary image representing the union. */
    cv::Mat union_set;
    /** @brief The size of the intersection set (nonzero element count). */
    double intersection_size;
    /** @brief The size of the union set (nonzero element count). */
    double union_size;
    /** @brief The IOU measure. */
    double iou;
    /**
     * @brief Initialize sets to a defined size, and values to NaN.
     *
     * @param rows The row count of the set.
     * @param cols The column count of the set.
     * @param type The data type of the set.
     */
    IOUVal(const int rows, const int cols, const int type);
  };  // struct IOUVal

  /**
   * @brief Compute the IOU for two sets represented by binary images.
   *
   * @param binary_image1 The first set.
   * @param binary_image2 The second set.
   *
   * @return The intersection and union sets as well as their IOU.
   */
  static IOUVal IOU(const cv::Mat& binary_image1, const cv::Mat& binary_image2);

  /**
   * @brief Update track information with given timestamp and contour.
   *
   * @param timestamp The measurement time to record for the track.
   * @param track The track to updated.
   * @param contour_info The contour information for the updated track.
   */
  static void updateTrack(const double timestamp, Track& track,
                          ContourInfo& contour_info);

  /**
   * @brief Find the nearest matching track to the given contour and compute
   * track information, including image plane dynamics.
   *
   * @param timestamp The timestamp at which the association is performed.
   * @param contour_info The contour information to match.
   *
   * @return The nearest matching id, or INVALID_ID if none is found.
   */
  Id computeTrackAssociation(const double timestamp, ContourInfo& contour_info);

  /**
   * @brief Find contours to focus on based on hueristics. This operation does
   * not preserve hierarchy information.
   *
   * @param frame_info The raw frame info.
   *
   * @return The frame info satisfying filter heuristics.
   */
  FrameInfo filterFrame(const FrameInfo& frame_info) const;

  /**
   * @brief Compute the next available unique id.
   *
   * @return The unique id.
   */
  Id getNextId();

  /**
   * @brief Randomly assign colors to tracks for visualization.
   *
   * @return An integer corresponding to a color.
   */
  int getNextColor();

  /**
   * @brief Compute all contour info from the given edge detection.
   *
   * @param timestamp The absolute timestamp of the frame info.
   * @param edges The image edge detections.
   *
   * @return A frame info object.
   */
  FrameInfo computeFrameInfo(const double timestamp,
                             const cv::Mat& edges) const;

  /** @brief Invalid value for track id. */
  static const Id INVALID_ID = -1;
  /** @brief Get the next color index. */
  int next_color_;
  /** @brief Storage for next available unique id. */
  Id next_id_;
  /** @brief The tracker parameters. */
  Params params_;
  /** @brief Associative container of id to tracks. */
  Tracks tracks_;
  /** @brief Buffer for storing a frame history. */
  boost::circular_buffer<FrameInfo> frame_info_buffer_;
};  // class ConnectedComponentTracker

}  // namespace maeve_automation_core
