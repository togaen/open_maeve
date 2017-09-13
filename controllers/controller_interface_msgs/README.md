# README #

This package defines generic interface messages between nodes that generate
control commands and nodes that send actuation commands to robot hardware.

## Messages ##

* Command2D.msg: A generic two dimensional control command message.

## Libraries ##

* Command2D\_Manager: This is a small client library that nodes can use to retrieve Command2D messages from the ROS graph. The advantage is that it correctly and transparently handles the 'lazy\_publishing' flag of the message.
