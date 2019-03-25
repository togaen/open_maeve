#
# This message describes a generic 2D control command.
#

# Message header.
Header header

# If true, the control should be applied until the next message is received.
# If false, the control should only be applied for a single cycle.
bool sticky_control

# X axis command. 
float64 x

# Y axis command.
float64 y
