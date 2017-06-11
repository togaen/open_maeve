#pragma once

#include "maeve_automation_core/ros_parameter_loading/params_base.h"

namespace maeve_automation_core {

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

  /// \brief alternative representation; computed after loading above params.
	/// \{
	double x_pos;
	double y_pos;
	double width;
	double height;
	/// \}
};  // struct BoundingBoxParams

}  // namespace maeve_automation_core

