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

#include <string>
#include <vector>

#include "open_maeve/maeve_expansion_segmentation/connected_component_tracking.h"
#include "open_maeve/ros_parameter_loading/ros_parameter_loading.h"

namespace open_maeve {

/** @brief Parameter object to load ROS params.*/
struct MaeveExpansionSegmentationParams : public ParamsBase {
  /**
   * @brief Container for parameters used during spatial feature detection.
   */
  struct SpatialParams {
    /** @brief The min edge pixel value threshold. */
    int edge_min;
    /** @brief The max edge pixel value threshold. */
    int edge_max;
    /** @brief The aperture size for edge Sobel operator. */
    int edge_aperture;
    /** @brief The aperture size for median blur operator. */
    int blur_aperture;
    /**
     * @brief Constructor. Initialize values to bad values.
     */
    SpatialParams();
    /**
     * @brief Run sanity checks on the parameters loaded into this object.
     *
     * @return True if sanity checks pass; otherwise false.
     */
    bool valid() const;
  };  // struct SpatialEdgeParams

  /**
   * @brief Container for parameter used during temporal feature detection.
   */
  struct TemporalParams {
    /** @brief History length to use for background subtraction. */
    int history;
    /** @brief Distance threshold to use for background subtraction. */
    int threshold;
    /** @brief Whether to detection shadows or not. */
    bool shadows;
    /**
     * @brief Constructor. Initialize values to bad values.
     */
    TemporalParams();
    /**
     * @brief Run sanity checks on the parameters loaded into this object.
     *
     * @return True if sanity checks pass; otherwise false.
     */
    bool valid() const;
  };  // struct TemporalEdgeParams

  /**
   * @brief Parameters for applying morphological operations.
   */
  struct MorphologicalParams {
    /** @brief The structuring element to use (0 - rectangle, 1 - ellipse). */
    int element_type;
    /** @brief The width of the window containing the structuring element. */
    int window_width;
    /** @brief The height of the window containing the structuring element. */
    int window_height;
    /**
     * @brief Constructor. Initialize value to bad values.
     */
    MorphologicalParams();
    /**
     * @brief Run sanity checks on the parameters loaded into this object.
     *
     * @return True if sanity checks pass; otherwise false.
     */
    bool valid() const;
  };  // struct MorphologicalParams

  /** @brief Connected component tracking parameters. */
  ConnectedComponentTracker::Params connected_component_params;

  /** @brief Spatial feature detection parameters. */
  SpatialParams spatial_params;

  /** @brief Temporal feature detection parameters. */
  TemporalParams temporal_params;

  /** @brief Dilation operator parameters. */
  MorphologicalParams dilation_params;

  /** @brief Erosion operator parameters. */
  MorphologicalParams erosion_params;

  /** @brief The list of morphological operation to apply in order. */
  std::vector<int> morpho_operations;

  /** @brief The image sequence topic. */
  std::string camera_topic;

  /** @brief Whether to enable vizualization of intermediate results. */
  bool enable_viz;

  /** @brief Output topic of temporal edge detection. */
  std::string viz_te_topic;

  /** @brief Output topic of spatial edge detection. */
  std::string viz_se_topic;

  /** @brief Output topic of AND image operation. */
  std::string viz_AND_topic;

  /**
   * @copydoc ParamsBase::ParamsBase()
   */
  __attribute__((warn_unused_result)) bool load(
      const ros::NodeHandle& nh) override;
};  // struct FeatureFlowParams

}  // namespace open_maeve
