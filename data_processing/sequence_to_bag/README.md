# README #

This package contains various utilities for converting data sequences to bag files. Currently two data sequence formats are supported:

* Parallel Domain
* Karlsruhe Dataset

The two usages are detailed below.

## Karlsruhe Dataset: Stereo Video Sequences + rough GPS Poses ##

This utility converts data sets from the [Karlsruhe Stereo Video Sequences + rough GPS Poses](http://www.cvlibs.net/datasets/karlsruhe_sequences/) collection into bag files. The stereo images are recorded to bag along with camera calibration information and the GPS and IMU poses. All messages are timestamped with the recorded time.

Visit the [dataset web page](http://www.cvlibs.net/datasets/karlsruhe_sequences/) for more information about the data format.

Bag files generated with this tool can be used with the ROS package [stereo\_image\_proc](https://wiki.ros.org/stereo_image_proc) to generate point clouds and disparity images.

### Usage ###

Issue the following command to see usage:

    rosrun sequence_to_bag karlsruhe_dataset --help

This will display a list of options as below:

    Available options are listed below. Arguments without default values are required:
      -h [ --help ]                      Print help and exit.
      -d [ --data-set-path ] arg         Absolute path to the data set directory.
      -o [ --bag-output-dir ] arg        Absolute path to the directory for the output bag file.
      -c [ --camera-name ] arg (=stereo) Frame name to use for the stereo camera image stream.
      -g [ --global-name ] arg (=world)  Name to use for the global frame in which the IMU measures motion.
      -i [ --imu-name ] arg (=imu)       Name to use for the local IMU frame as it moves in the global frame.

## Parallel Domain ##

This utility for converts image segmentation sequence data sets into bag files. Usage and the specification for the data set structure is given below.

The utility looks for camera\_info.yaml in data-set-path in order to record camera parameters to the bag file. If the file doesn't exist, the utility synthesizes parameters with unit focal lengths, centered optical center, and no distortion.

### Usage ###

Issue the following command to see usage:

    rosrun sequence_to_bag parallel_domain --help

This will display a list of options as below:

    Available options are listed below. Arguments without default values are required:
      -h [ --help ]                                    Print help and exit.
      -d [ --data-set-path ] arg                       Absolute path to the data set.
      -o [ --bag-output-dir ] arg                      Absolute path to the directory for the output bag file.
      -r [ --raw-image-camera ] arg (=raw)             Camera name to use for the raw image stream.
      -s [ --segmented-image-camera ] arg (=segmented) Camera name to use for the segmented image stream.

### Image Segmentation Sequence Data Set ###

The data set consists of a sequence of image frames and corresponding sequence of segmentation frames. The image frames are the images that the camera creates, and the segmentation frames are per-pixel labeled versions of the camera frames. The data set is a directory containing [YAML](http://www.yaml.org/start.html) files and two sub-directories containing the camera and segmented images:

    data-set
     |
     |- meta.yaml
     |- segmentation_taxonomy.yaml
     |- camera-frames
        |- frame-01.png
        |- frame-02.png
        |- etc.
     |- segmented-frames
        |- frame-01.png
        |- frame-02.png
        |- etc.

The camera and segmentation frames shall have the same dimensions, and there shall be a one-to-one correspondence between the files indicated by the number in their names. The numbering shall be integral and sequential. The number intended to act as the index shall be the last sub-string in the basename. For example, the following files have indices 3, 4, and 9, respectively:

    path/to/file-003.png
    path/to/01-file-004.png
    path/to/9.png

It is not required that indexing be gapless nor that it start with one (i.e., "1, 2, 3" in the above example instead of "3, 4, 9"), but it is encouraged. The rest of the filename is arbitrary and only useful for human readability.

The meta file shall have the format given below, and provides information useful for data processing purposes:

```yaml
# meta.yaml

# Meta information about image/feature data set.
sequence_meta:
  # Brief description of the data set.
  description: "This is an example data set."
  
  # Name of the data set; should also be name of root folder.
  name: "data-set"

  # Name of the raw image sequence for the data set; should also be the name of the folder containing the images.
  raw: "camera-frames"

  # Name of the segmented image sequence for the data set; should also be the name of the folder containing the images.
  segmented: "segmented-frames"

  # Used to determine time delta between frames.
  fps: 30.0
```
For a description of label\_map.yaml, see the README in the
segmentation\_taxonomy package.

