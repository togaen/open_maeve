# README #

This package contains convenience launch files for starting various cameras.
Currently supported supported are:

* webcam.launch: A web camera launch file that uses the [cv\_camera](http://wiki.ros.org/cv_camera) package.
* raspicam\_node.launch: A Raspberry Pi camera launch file that launches a pre-configured [raspicam\_node](https://github.com/fpasteau/raspicam_node).
* raspicam\_rectify.launch: Loads the image\_proc packages and rectifies the uncompressed output from the raspicam.launch configured camera
* raspicam.launch: A convenience launch file that loads raspicam\_node.launch, republishes the output as uncompressed, and then runs raspicam\_rectify.launch
* vid\_player.launch: Convenience launch file for streaming video files to the ROS graph; use the 'vid\_file' argument to pass it a path to the video file to play.
