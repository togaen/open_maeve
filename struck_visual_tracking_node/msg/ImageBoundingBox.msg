#
# This message defines a bounding box on an image
#

std_msgs/Header header

int64 image_width
int64 image_height

#
# Extents in image plane coordinates: (0, 0) upper left corner, x from left to right, y from top to bottom
#

float64 x_min
float64 x_max
float64 y_min
float64 y_max

