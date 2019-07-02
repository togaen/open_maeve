#!/bin/bash
# Script to run clang format recursively on C++ files under the current directory
# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

find . -iname *.h -o -iname *.hpp -o -iname *.cpp -o -iname *.c -o -iname *.cc -o -iname *.hh | xargs clang-format -i
