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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <exception>

namespace maeve_automation_core {
/**
 * @brief The Image Space Potential field class.
 *
 * @tparam T_Tx Functor type defining the potential transform.
 */
template <typename T_Tx>
class ImageSpacePotentialField {
 public:
  /**
   * @brief Exception to throw if initialization fails.
   */
  class ISPInvalidInputTypeExcpetion : public std::exception {
    /**
     * @brief Explain the input error the caused initialization failure.
     *
     * @return The error string.
     */
    const char* what() const noexcept override;
  };  // class ISPInvalidInputTypeException

  /**
   * @brief Constructor: Create an Image Space Potential field by transforming
   * the given ttc field.
   *
   * @param ttc_field The ttc field.
   * @param tx The pixel value -> potential value transform.
   */
  ImageSpacePotentialField(const cv::Mat& ttc_field, const T_Tx& tx);

  /**
   * @brief Accessor for the Image Space Potential field.
   *
   * @return A const ref to the potential field. This is a 2-channel
   * double-valued matrix where the first channel is potential value and second
   * channel is its time derivative.
   */
  const cv::Mat& field() const;

 private:
  /**
   * @brief Apply a potential transform to a scalar field.
   */
  class ApplyTransform : public cv::ParallelLoopBody {
   public:
    /**
     * @brief Constructor: Set field and transform references.
     *
     * @param field Reference to the field being transformed.
     * @param tx Reference to the transform functor.
     */
    ApplyTransform(cv::Mat& field, const T_Tx& tx);

    /**
     * @brief Apply transform to pixels in range r.
     *
     * @param r The range of pixels to apply transform to.
     */
    void operator()(const cv::Range& r) const override;

   private:
    /** @brief Reference to the scalar field being transformed. */
    cv::Mat& field_ref_;
    /** @brief Reference to the transform. */
    const T_Tx& tx_;
  };  // class ApplyTransform

  /** @brief Storage for the Image Space Potential field. */
  cv::Mat field_;
};  // class ImageSpacePotentialField

#include "impl/isp.impl.h"

}  // namespace maeve_automation_core
