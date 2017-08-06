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

namespace maeve_automation_core {

/**
 * @brief Tracker class to perform simple connected component tracking.
 */
class ConnectedComponentTracker {
 public:
  /** @name Convenience typedefs */
  /*@{
   */
  typedef std::vector<cv::Point> Contour;
  typedef std::vector<cv::Vec4i> ContourHierarchy;
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
    int IOU_threshold;
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
    /**
     * @brief Constructor: initialize ignore to false and area to NaN.
     */
    ContourInfo();
  };  // struct ContourInfo

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
   * @param edges The edge detection.
   *
   * @return True if frame buffer has reached size; otherwise false.
   */
  bool addEdgeFrame(const cv::Mat& edges);

  /**
   * @brief Get a const reference to the frame info buffer.
   *
   * @return A const ref to the frame info buffer.
   */
  const boost::circular_buffer<FrameInfo>& getFrameInfoBuffer() const {
    return frame_info_buffer_;
  }

 private:
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
  size_t getNextId();

  /**
   * @brief Compute all contour info from the given edge detection.
   *
   * @param edges The image edge detections.
   *
   * @return A frame info object.
   */
  FrameInfo computeFrameInfo(const cv::Mat& edges) const;

  /** @brief Storage for next available unique id. */
  size_t next_id;
  /** @brief The tracker parameters. */
  Params params_;
  /** @brief Associative container of id to tracks. */
  std::unordered_map<int, cv::Mat> tracks_;
  /** @brief Buffer for storing a frame history. */
  boost::circular_buffer<FrameInfo> frame_info_buffer_;
};  // class ConnectedComponentTracker

}  // namespace maeve_automation_core
