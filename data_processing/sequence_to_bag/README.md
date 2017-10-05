# README #

This package is a command line utility for converting sequence image
segmentation sequence data sets into bag files. The specification for the data
set structure is given below.

## Image Segmentation Sequence Data Set ##

The data set consists of a sequence of image frames and corresponding sequence
of segmentation frames. The image frames are the images that the camera creates,
and the segmentation frames are per-pixel labeled versions of the camera frames.
The data set is a directory containing a [YAML](http://www.yaml.org/start.html)
file and two sub-directories containing the camera and segmented images:

    data-set
     |
     |- meta.yaml
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
in their names. The text portion of their names is arbitrary and only useful for
human readability.

The meta file shall have the format given below. The label map is defined by
string/RGB-value pairs, where the pairs are defined by each data set. A small
example map is defined below:

```yaml
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

  # In the segmented images, the objects are identified by their pixel RGB values.
  label_map:
    road:           [123, 123, 123]
    lane_marker:    [169, 169, 169]
    lane_boundary:  [143, 142, 89]
```
