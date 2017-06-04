#pragma once

#include <string>
#include <sstream>

#include "ros/ros.h"

#include "maeve_automation_core/struck_visual_tracking/Config.h"

#define CHECK_STRICTLY_POSITIVE(var)\
	if (var <= 0) {\
		ROS_ERROR_STREAM(#var << " <= 0: check failed");\
		return false;\
	}

#define CHECK_NONEMPTY(var) \
	if (var.empty()) {\
		ROS_INFO_STREAM(#var << ".empty(): check failed");\
		return false;\
	}

struct StruckVisualTrackingParams {
	/// \brief Stream overload
  friend std::ostream& operator<<(std::ostream &os, const StruckVisualTrackingParams& params);

	/// \brief Load parameters from parameter server.
  bool load(const ros::NodeHandle& nh);

	/// \brief Convert this parameter object to a STRUCK config object.
  Config toStruckConfig() const;

  /// \brief Check that a Struck config object has its members set to reasonable values.
	/// \return True if params seem okay; otherwise false.
	static bool SanityCheckStruckConfig(const Config& c);

	// topic name for camera images
	std::string camera_topic;

	// quiet mode disables all visual output (for experiments).
	bool quietMode;

  // debug mode enables additional drawing and visualization.
	bool debugMode;

  // base path for video sequences.
	std::string sequenceBasePath;

  // path for output results file.
  // comment this out to disable output.
  std::string resultsPath;

  // video sequence to run the tracker on.
  // comment this out to use webcam.
	std::string sequenceName;

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
		if (c.sequenceName.empty() != c.sequenceBasePath.empty()) {
			ROS_ERROR_STREAM("sequenceName.empty() != sequenceBasePath.empty(): check failed");
			return false;
		}
	  CHECK_STRICTLY_POSITIVE(c.frameWidth);
    CHECK_STRICTLY_POSITIVE(c.frameHeight);
	  CHECK_STRICTLY_POSITIVE(c.searchRadius);
	  CHECK_STRICTLY_POSITIVE(c.svmC);
	  CHECK_STRICTLY_POSITIVE(c.svmBudgetSize);
    return true;
  }

	/// \brief Human-readable string of parameters loaded by load() function.
	std::stringstream loaded_param_set;
};  // struct StruckVisualTrackingParams

