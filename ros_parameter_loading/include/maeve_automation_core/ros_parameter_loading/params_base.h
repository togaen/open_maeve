#pragma once

#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>

#define CHECK_STRICTLY_POSITIVE(var)\
	if (var <= 0) {\
		ROS_ERROR_STREAM(#var << " <= 0: check failed");\
		return false;\
	}

#define CHECK_NONEMPTY(var) \
	if (var.empty()) {\
		ROS_ERROR_STREAM(#var << ".empty(): check failed");\
		return false;\
	}

#define LOAD_PARAM(var) \
	if (!nh.getParam(#var, var)) {\
		ROS_ERROR_STREAM("Failed to load parameter '" << #var << "'");\
		return false;\
	}\
  {\
	 std::stringstream ss;\
   ss << #var << ": " << var << "\n";\
   loaded_param_set += ss.str();\
	}

struct ParamsBase {
	/// \brief Stream overload
  friend std::ostream& operator<<(std::ostream &os, const ParamsBase& params) {
    return os << params.loaded_param_set;
	}

	/// \brief Load parameters from parameter server.
  virtual bool load(const ros::NodeHandle& nh) = 0;

	protected:

	/// \brief Human-readable string of parameters loaded by load() function.
	std::string loaded_param_set;
};  // struct StruckVisualTrackingParams

