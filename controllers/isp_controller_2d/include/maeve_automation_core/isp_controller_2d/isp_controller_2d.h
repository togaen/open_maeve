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

#include "maeve_automation_core/isp_controller_2d/control_command.h"
#include "maeve_automation_core/isp_field/potential_transforms.h"

namespace maeve_automation_core {
class ISP_Controller2D {
 public:
  /**
   * @brief Container for controller parameters.
   */
  struct Params {
    /**
     * @brief Container for guidance gain values.
     */
    struct GuidanceGains {
      /** @brief Apply this gain to the throttle guidance horizon. */
      double throttle;
      /** @brief Apply this gain to the yaw guidance horizon. */
      double yaw;
      /** @brief Apply this gain to the control set guidance horizon. */
      double control_set;
      /**
       * @brief Whether the parameter values are valid.
       *
       * @return True if the values are valid; otherwise false.
       */
      __attribute__((warn_unused_result)) bool valid() const;
      /**
       * @brief Constructor: initialize to invalid values.
       */
      GuidanceGains();
      /**
       * @brief Constructor: explicit initialization.
       *
       * @param t The throttle gain \in (0, +\infty).
       * @param y The yaw gain \in (0, +\infty).
       * @param c The control_set gain \in (0, +\infty).
       */
      GuidanceGains(const double t, const double y, const double c);
    };  // struct GuidanceGains

    /**
     * @brief Container for erosion parameters.
     */
    struct ErosionKernel {
      /** @brief Min filter kernel width. */
      double width;
      /** @brief Min filter kernel height. */
      double height;
      /** @brief Min filter kernel horizon line from 0 (top) to 1 (bottom). */
      double horizon;
      /**
       * @brief Whether the parameter values are valid.
       *
       * @return True if they are valid; otherwise false.
       */
      __attribute__((warn_unused_result)) bool valid() const;
      /**
       * @brief Constructor: initialize to invalid values.
       */
      ErosionKernel();
      /**
       * @brief Constructor: explicit initialization.
       *
       * @param w Width of the kernel \in [0, 1] as a portion of image width.
       * @param ht Height of the kernel \in [0, 1] as a portion of image height.
       * @param hr Horizon line to apply kernel to \in [0, 1] as a portion of
       * image height.
       */
      ErosionKernel(const double w, const double ht, const double hr);
    };  // struct ErosionKernel

    struct HorizonDecay {
      /** @brief Yaw guidance left decay \in [0, 1]. */
      double left;
      /** @brief Yaw guidance right decay \in [0, 1]. */
      double right;
      /**
       * @brief Whether the parameter values are valid.
       *
       * @return True if the values are valid; otherwise false.
       */
      __attribute__((warn_unused_result)) bool valid() const;
      /**
       * @brief Constructor: initialize to invalid values.
       */
      HorizonDecay();
      /**
       * @brief Constructor: explicit initialization.
       *
       * @param l The left decay parameter \in [0, 1].
       * @param r The right decay parameter \in [0, 1].
       */
      HorizonDecay(const double l, const double r);
    };  // struct HorizonDecay

    /** @brief Guidance gains. */
    GuidanceGains guidance_gains;
    /** @brief Erosion kernel parameters. */
    ErosionKernel erosion_kernel;
    /** @brief Yaw horizon decay parameters. */
    HorizonDecay yaw_decay;
    /** @brief Camera focal length along x (pixels). */
    double focal_length_x;
    /** @brief The x-coordinate of the camera principal point (pixels). */
    double principal_point_x;
    /** @brief Proportional gain. */
    double K_P;
    /** @brief Derivative gain. */
    double K_D;
    /** @brief Potential delta must exceed this to trigger control change. */
    double potential_inertia;
    /** @brief Shape parameters used for safe control computation. */
    ShapeParameters shape_parameters;
    /**
     * @brief Whether the parameters all have valid values.
     *
     * @return True if values are valid; otherwise false.
     */
    __attribute__((warn_unused_result)) bool valid() const;
    /**
     * @brief Constructor: initialize to invalid values.
     */
    Params();
    /**
     * @brief Constructor: explicit initialization.
     *
     * @param sp The shape parameters object.
     * @param ek The erosion kernel object.
     * @param yd The yaw decay object.
     * @param gg The guidance gains object.
     * @param fx The camera focal length along x (pixels).
     * @param px The x-coordinate of the camera principal point (pixels).
     * @param kp Proportional gain for control projection.
     * @param kd Derivative gain for control projection.
     * @param pi Potential inertia to overcome to change direction.
     */
    Params(const ShapeParameters& sp, const ErosionKernel& ek,
           const HorizonDecay& yd, const GuidanceGains& gg, const double fx,
           const double px, const double kp, const double kd, const double pi);
  };  // struct Params

  /**
   * @brief Enumerate control structures that can be inspected.
   */
  enum class ControlStructure {
    CONTROL_HORIZON,
    ERODED_CONTROL_HORIZON,
    INVALID
  };

  /**
   * @brief Constructor: do not mark as initialized.
   */
  ISP_Controller2D();

  /**
   * @brief Constructor: initialize with shape parameters.
   *
   * @param params Parameters for control computation from ISP field.
   */
  explicit ISP_Controller2D(const Params& params);

  /**
   * @brief For a given ISP field compute a selective determinism control.
   *
   * @note For reasonable results, the desired control commands should be in the
   * range [-1, 1] indicating minimum and maximum actuation values,
   * respectively.
   *
   * @param ISP The input ISP field.
   * @param u_d The desired control command.
   *
   * @return The computed control command with each member in the range [-1, 1]
   * indicating minimum and maximum actuation, respectively.
   */
  ControlCommand SD_Control(const cv::Mat& ISP, const ControlCommand& u_d);

  /**
   * @brief Whether the controller has been initialized with its parameters.
   *
   * @return True if the controller is initialized; otherwise false.
   */
  bool isInitialized() const;

  /**
   * @brief Accessor for most recently computed horizon data structures.
   *
   * @return A const ref to the desired horizon structure.
   */
  const cv::Mat& inspectHorizon(const ControlStructure cs) const;

  /**
   * @brief Get string representation of control structure enum.
   *
   * @param cs The control structure enum.
   *
   * @return The string version of 'cs'.
   */
  static std::string controlStructureToString(const ControlStructure cs);

  /**
   * @brief Get control sequence enum corresponding to given string
   *
   * @param str
   *
   * @return
   */
  static ControlStructure stringToControlStructure(const std::string& str);

 private:
  /** @brief Whether the object has been initialied. */
  bool init_;
  /** @brief Controller parameters. */
  Params p_;
  /** @brief Storage for intermediate control horizon computation. */
  cv::Mat h_;
  /** @brief Storage for intermediate eroded control horizon computation. */
  cv::Mat eroded_h_;
  /** @brief Projection function onto control space. */
  PotentialTransform<ConstraintType::SOFT> C_u_;
};  // class ISP_Controller2D

/**
 * @brief Overload output stream operator for controller parameter set.
 *
 * @param o The output stream.
 * @param sp The controller parameters object.
 *
 * @return The stream.
 */
std::ostream& operator<<(std::ostream& o, const ISP_Controller2D::Params& p);

/**
 * @brief Overload output stream operator for controller parameter set.
 *
 * @param o The output stream.
 * @param sp The controller parameters object.
 *
 * @return The stream.
 */
std::ostream& operator<<(std::ostream& o,
                         const ISP_Controller2D::Params::HorizonDecay& hd);

/**
 * @brief Overload output stream operator for controller parameter set.
 *
 * @param o The output stream.
 * @param sp The controller parameters object.
 *
 * @return The stream.
 */
std::ostream& operator<<(std::ostream& o,
                         const ISP_Controller2D::Params::GuidanceGains& gg);

/**
 * @brief Overload output stream operator for controller parameter set.
 *
 * @param o The output stream.
 * @param sp The controller parameters object.
 *
 * @return The stream.
 */
std::ostream& operator<<(std::ostream& o,
                         const ISP_Controller2D::Params::ErosionKernel& ek);
}  // namespace maeve_automation_core
