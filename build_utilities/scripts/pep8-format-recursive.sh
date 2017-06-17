#!/bin/bash
# Script to run autopep8 format recursively on python files under the current directory
# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

# E266: too many leading ‘#’ for block comment; ignored because it breaks Doxygen docs
autopep8 --in-place --aggressive --aggressive --recursive --ignore=E266 .
