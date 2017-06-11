#!/bin/bash
# Script to generate Eclipse files for ROS packages. See: http://wiki.ros.org/IDEs#Eclipse
# Copyright (C) 2017  Open Source Robotics Foundation, Inc. - All Rights Reserved
# Permission to copy and modify is granted under the Creative Commons Attribution 3.0 license

#
# Step 1: catkin build --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=RelWithDebInfo
# Step 2: Run this script
# Step 3: Import package build directory into Eclipse as an existing project
#

PWD=`pwd`
ROOT=~/catkin_ws
cd $ROOT
cd build
echo `pwd`
for PROJECT in `find $ROOT -name .project`; do
    DIR=`dirname $PROJECT`
    echo $DIR
    cd $DIR
    awk -f $(rospack find mk)/eclipse.awk .project > .project_with_env && mv .project_with_env .project
done
cd $PWD
