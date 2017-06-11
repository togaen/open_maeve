#pragma once

#include "maeve_automation_core/struck_visual_tracking/Config.h"
#include "maeve_automation_core/ros_parameter_loading/params_base.h"

struct StruckVisualTrackingParams : public ParamsBase {
	/// \brief Load parameters from parameter server.
  bool load(const ros::NodeHandle& nh) override;

	/// \brief Convert this parameter object to a STRUCK config object.
  Config toStruckConfig() const;

  /// \brief Check that a Struck config object has its members set to reasonable values.
	/// \return True if params seem okay; otherwise false.
	static bool SanityCheckStruckConfig(const Config& c);

	// topic name for camera images
	std::string camera_topic;

	// topic to publish tracker images to
  std::string tracker_image_topic;

  // topic to publish tracker bounding boxes to
	std::string tracker_bb_topic;

  // enable visualization topic
	bool enable_viz;

	// quiet mode disables all visual output (for experiments).
	bool quietMode;

  // debug mode enables additional drawing and visualization.
	bool debugMode;

  // frame size for use during tracking.
  // the input image will be scaled to this size.
	int frameWidth;
	int frameHeight;

  // seed for random number generator.
	int seed;

  // tracker search radius in pixels.
	int searchRadius;

  // SVM regularization parameter.
	double svmC;

  // SVM budget size (0 = no budget).
	int svmBudgetSize;

  // image features to use.
  // format is: feature kernel [kernel-params]
  // where:
  //   feature = haar/raw/histogram
  //   kernel = gaussian/linear/intersection/chi2
  //   for kernel=gaussian, kernel-params is sigma
  // multiple features can be specified and will be combined
	std::string feature;

	private:
	/// \brief Check common values of Struck and MA param types.
	/// \return True if seems okay; otherwise false.
	template <typename T>
  static bool SanityCheckConfig(const T& c) {
	  CHECK_STRICTLY_POSITIVE(c.frameWidth);
    CHECK_STRICTLY_POSITIVE(c.frameHeight);
	  CHECK_STRICTLY_POSITIVE(c.searchRadius);
	  CHECK_STRICTLY_POSITIVE(c.svmC);
	  CHECK_STRICTLY_POSITIVE(c.svmBudgetSize);
    return true;
  }
};  // struct StruckVisualTrackingParams

