#!/usr/bin/env python
# Copyright (C) 2017 Sebastian Klose - All Rights Reserved
# Permission to copy and modify is granted under the GPLv3 license

import os
import json

## @package merge_compile_commands
# Helper script for YouCompleteMe vim plugin.
#
# When using catkin tools with ROS, the compile_commands.json files are
# generated per package. This script collects them all and dumps them into a
# single file at the root of the workspace. This is needed for the
# YouCompleteMe vim plugin.

workspace_path = os.path.expanduser('~') + '/catkin_ws'
compile_command_filename = 'compile_commands.json'
root = workspace_path
output_file = root + '/' + compile_command_filename
head = list()
for root, folder, files in os.walk(root):
    if compile_command_filename in files:
        with open(os.path.join(root, compile_command_filename), 'r') as handle:
            head += json.load(handle)

print 'Outputting to: ' + output_file
with open(output_file, 'w') as output_file_handle:
    json.dump(head, output_file_handle, indent=0)
