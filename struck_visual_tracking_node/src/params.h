#pragma once

#include <string>
#include <sstream>

#include <ros/ros.h>


struct StruckVisualTrackingParams {
  friend std::ostream& operator<<(std::ostream &os, const StruckVisualTrackingParams& params);

	/// \brief Load parameters from parameter server.
  bool load(const ros::NodeHandle& nh);

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
	std::stringstream debug_out;
};  // struct StruckVisualTrackingParams

