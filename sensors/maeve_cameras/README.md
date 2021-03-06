# README #

This package contains convenience launch files for starting various cameras.
Currently supported supported are:

* webcam.launch: A web camera launch file that uses the [cv\_camera](http://wiki.ros.org/cv_camera) package.
* raspicam\_node.launch: A Raspberry Pi camera launch file that launches a pre-configured [raspicam\_node](https://github.com/fpasteau/raspicam_node).
* raspicam\_post\_process.launch: Loads the image\_proc packages and rectifies the uncompressed output from the raspicam.launch configured camera, and republishes it to the full spectrum of transport types.
* vid\_player.launch: Convenience launch file for streaming video files to the ROS graph; use the 'file' command-line launch file argument to pass it a path to the video file to play. Optionally specify the frame rate with the 'fps' command-line launch file argument (default fps:=30).
