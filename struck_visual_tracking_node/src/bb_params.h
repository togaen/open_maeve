#pragma once

#include "maeve_automation_core/ros_parameter_loading/params_base.h"

struct BoundingBoxParams : public ParamsBase {
	/// \brief Load parameters from parameter server.
  bool load(const ros::NodeHandle& nh) override;

	/// \brief Bounding box specification.
	/// \{
	double bb_x_min;
	double bb_x_max;
	double bb_y_min;
	double bb_y_max;
	/// \}
};  // struct BoundingBoxParams
