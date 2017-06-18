#
# This message describes a generic 2D control command. By convention, command
# values are confined to the range [-1, 1]. The consumer of the message is
# expected to perform any scaling of the values that may be necessary.
#

# Message header.
Header header

# If true, the listener should only expect message updates when the control
# command changes (e.g. when generated from joystick input).
# If false, the listener should messages to be published at a regular interval.
bool lazy_publishing

# X axis command \in [-1, 1].
float64 x

# Y axis command \in [-1, 1].
float64 y
