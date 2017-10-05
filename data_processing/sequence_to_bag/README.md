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

```
