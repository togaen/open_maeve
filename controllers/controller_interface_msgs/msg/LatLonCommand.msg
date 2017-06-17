#
# This message describes a 2D lateral/longitudinal control command.
#

# Message header.
Header header

# If true, the listener should only expect message updates when the control
# command changes (e.g. when generated from joystick input).
# If false, the listener should messages to be published at a regular interval.
bool lazy_publishing

# Lateral command \in [0, 1].
float64 lat_cmd

# Longitudinal command \in [0, 1].
float64 lon_cmd
