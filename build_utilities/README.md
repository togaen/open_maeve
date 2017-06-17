# README #

This package contains shell and python scripts that help maintain build
infrastructure. The scripts can be called with `rosrun` or directly, and they
assume standard catkin workspace layout, i.e., `~/catkin_ws`.

The scripts are:

* `clang-format-recursive.sh`: Perform a recursive, in-place clang format on
C++ source files starting in the current working directory.
* `make-eclipse-projects.sh`: Generate project files for Eclipse IDE.
* `merge-compile-commands.py`: When using `catkin tools` compile\_commands.json
files are generated per package. This script combines them all into a single
file at the workspace root.

See individual scripts for further documentation. 

