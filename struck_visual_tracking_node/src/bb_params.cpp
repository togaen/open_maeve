#include "bb_params.h"

bool BoundingBoxParams::load(const ros::NodeHandle& nh) {
  LOAD_PARAM(bb_x_min);
  LOAD_PARAM(bb_x_max);
  LOAD_PARAM(bb_y_min);
	LOAD_PARAM(bb_y_max);

	CHECK_GT(bb_x_max, bb_x_min);
	CHECK_GT(bb_y_max, bb_y_min);

	return true;
}

