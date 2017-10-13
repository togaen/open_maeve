# README #

TODO

## Segmentation Taxonomy ##

The label map file defines string/RGB-value pairs that enable interpretation of
the segmented images, where each label defines a class. A class with all
non-zero RGB values does not allow unique instances. A class with zero RGB
values allows unique instances by setting the zero values to non-zero values.

An example label map is given below: 

```yaml
# label_map.yaml

# This key should match the name specified in meta.yaml
data-set:
  # In the segmented images, the objects are identified by their pixel RGB values.
  label_classes:
    car:            [255, 0,   0]
    road:           [123, 123, 123]
    lane_marker:    [169, 169, 169]
    lane_boundary:  [143, 142, 89]

  label_instances: [
    [255, 1, 1],
    [255, 1, 2]
  ]
```
