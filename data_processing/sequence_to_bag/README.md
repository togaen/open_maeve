# README #

This package is a command line utility for converting sequence image
segmentation sequence data sets into bag files. The specification for the data
set structure is given below.

## Usage ##

To use the converter, issue a command like this:

    rosrun sequence_to_bag sequence_to_bag data-set-path bag-output-dir camera-image-topic-name segmented-image-topic-name


## Image Segmentation Sequence Data Set ##

The data set consists of a sequence of image frames and corresponding sequence
of segmentation frames. The image frames are the images that the camera creates,
and the segmentation frames are per-pixel labeled versions of the camera frames.
The data set is a directory containing a [YAML](http://www.yaml.org/start.html)
file and two sub-directories containing the camera and segmented images:

    data-set
     |
     |- meta.yaml
     |- label_map.yaml
     |- camera-frames
        |- frame-01.png
        |- frame-02.png
        |- etc.
     |- segmented-frames
        |- frame-01.png
        |- frame-02.png
        |- etc.

The camera and segmentation frames shall have the same dimensions, and there
shall be a one-to-one correspondence between the files indicated by the number
in their names. The numbering shall be integral, one indexed, and sequential.
The number intended to act as the index shall be the last sub-string in the
basename. For example, the following files have indices 3, 4, and 9,
respectively:

    path/to/file-003.png
    path/to/01-file-004.png
    path/to/9.png

It is not required that indexing be sequential nor that it start with one
(i.e., "1, 2, 3" in the above example instead of "3, 4, 9"), but it is
encouraged. The reset of the filename is arbitrary and only useful for human
readability.

The meta file shall have the format given below, and provides information useful
for data processing purposes:

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

The label map file defines string/RGB-value pairs that enable interpretation of
the segmented images. The map is defined for each data set. A small example
map is defined below:

```yaml
# label_map.yaml

# This key should match the name specified in meta.yaml
data-set:
  # In the segmented images, the objects are identified by their pixel RGB values.
  label_map:
    road:           [123, 123, 123]
    lane_marker:    [169, 169, 169]
    lane_boundary:  [143, 142, 89]
```
