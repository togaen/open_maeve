# README #

This package contains convenience launch files for starting various cameras.
Currently supported supported are:

* webcam.launch: A web camera launch file that uses the [cv\_camera](http://wiki.ros.org/cv_camera) package.
* raspicam.launch: A Raspberry Pi camera launch file that uses the [raspicam\_node](https://github.com/fpasteau/raspicam_node) package.

In addition, utility launch files include:

* raspicam\_republish\_raw.launch: This launch file republishes compressed images from the raspicam process as uncompressed images.
