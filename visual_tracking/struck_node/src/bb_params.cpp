#include "bb_params.h"

namespace maeve_automation_core {

bool BoundingBoxParams::load(const ros::NodeHandle& nh) {
  LOAD_PARAM(bb_x_min);
  LOAD_PARAM(bb_x_max);
  LOAD_PARAM(bb_y_min);
	LOAD_PARAM(bb_y_max);

	CHECK_GT(bb_x_max, bb_x_min);
	CHECK_GT(bb_y_max, bb_y_min);

  // Compute alternate representation.
	width = bb_x_max - bb_x_min;
	height = bb_y_max - bb_y_min;
	x_pos = bb_x_min + (width / 2.0);
	y_pos = bb_y_min + (height / 2.0);

	return true;
}

}  // namespace maeve_automation_core

