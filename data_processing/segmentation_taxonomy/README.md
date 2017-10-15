# README #

This package defines libraries for loading data set taxonomies from YAML files.
The YAML file specification is given below.

## Segmentation Taxonomy ##

The label map file defines string/RGB-value pairs that enable interpretation of
the segmented images, where each label defines a class. A class with all
non-zero RGB values does not allow unique instances. A class with zero RGB
values allows unique instances by setting the zero values to non-zero values.

## Schema ##

The taxonomy contains labels for object classes and unique IDs. The labels are
represented as RGB values in [0, 255]. The schema has the following rules:

1. The root key should match the data set name given in the meta.yaml file for the data set.
1. The root key has exactly three children: label\_classes, label\_instances, label\_instance\_classes
1. The label\_classes key has children where each has an RGB value, and the child key is the class name, and the child value is the class label.
1. The label\_instances is a list of RGB values that contains all unique IDs in the data set.
1. The label\_instance\_classes is a list of class name lists, where each list defines the class membership of the ID at the same index in label\_instances.

An example label map is given below: 

```yaml
# label_map.yaml

# This key should match the name specified in meta.yaml
data-set:
  # In the segmentations, the semantic classes are indicated by per-pixel RGB
  # values.
  label_classes:
    car:            [255, 0,   0]
    road:           [123, 123, 123]
    lane_marker:    [169, 169, 169]
    lane_boundary:  [143, 142, 89]
  # Each of these labels uniquely identifies an entity, or class instance, in
  # the segmentation.
  label_instances: [
    [255, 1, 1],
    [255, 1, 2]
  ]
  # These class names are taken from 'label_classes' and correspond, in order,
  # to the class instances above, e.g., the entity indicated by [255, 1, 1] has
  # class membership to 'lane_boundary' and 'lane_marker', and the entity
  # indicated by [255, 1, 1] has class membership to 'tree'.
  label_instance_classes: [
    ['lane_boundary', 'lane_marker'],
    ['tree']
  ]
```
