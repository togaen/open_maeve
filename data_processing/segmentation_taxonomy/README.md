# README #

This package defines libraries for loading data set taxonomies from YAML files.
The YAML file specification is given below.

## Segmentation Image ##

The segmentation image is a standard RGB image where each pixel value is
interpreted as a class or instance label. A pixel value that does not exist in
either set of class or instance lables is ignored.

A label map file defines RGB-valued labels that enable interpretation of the
segmentation images. These labels are used to define 'classes', which are
string/lable pairs, and 'instances' which are uniquely identified entites in the
segmentations.

An instance must have membership to at least one class, but not all classes must
have instances. This distinction is useful for cases where it may not be useful
to assign unique identifiers, e.g., uniquely identifying all instances of a
'lane\_marker' class may not be meaningful, whereas uniquely identifying all
instances of 'car' may be.

## Segmentation Schema ##

The taxonomy contains labels for object classes and unique IDs. The labels are
represented as RGB values in [0, 255]. The schema has the following rules:

1. The root key should match the data set name given in the meta.yaml file for the data set.
1. The root key has exactly three children: label\_classes, label\_instances, label\_instance\_classes
1. The label\_classes key has children where each has an RGB value, and the child key is the class name, and the child value is the class label.
1. The label\_instances is a list of RGB values that contains all unique IDs in the data set.
1. The label\_instance\_classes is a list of class name lists, where each list defines the class membership of the ID at the same index in label\_instances.
1. Labels used for classes and instances must be unique, i.e., the same label may not exist as both a class and an instance. Program behavior is undefined if this is violated.
1. RGB values for classes and instances can be specified as [min, max] ranges (see lane\_marker in the example below).

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
    lane_marker:    [[166, 168, 167], [170, 171, 170]]
    lane_boundary:  [143, 142, 89]
    building:       [123, 123, 123]
    infrastructure: [100, 100, 100]
  # Each of these labels uniquely identifies an entity, or class instance, in
  # the segmentation. Each instance belongs to at least one class from the
  # 'label_classes' set above, and class membership is defined in the
  # 'label_instances_classes' list below.
  label_instances: [
    [255, 1, 1],
    [255, 1, 2]
  ]
  # These class names are taken from 'label_classes' and correspond, in order,
  # to the class instances above, e.g., the entity indicated by [255, 1, 1] has
  # class membership to 'lane_boundary' and 'lane_marker', and the entity
  # indicated by [255, 1, 1] has class membership to 'tree'.
  label_instance_classes: [
    ['infrastructure', 'building'],
    ['tree']
  ]
```
