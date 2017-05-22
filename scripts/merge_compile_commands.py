#!/usr/bin/env python
import os
import json

workspace_path = os.path.expanduser('~') + '/catkin_ws'

compile_command_filename = 'compile_commands.json'
root = workspace_path
output_file = root + '/' + compile_command_filename
head = list()
for root, folder, files in os.walk(root):
    if compile_command_filename in files:
        with open(os.path.join(root, compile_command_filename), 'r') as handle:
            head += json.load(handle)

print output_file
with open(output_file, 'w') as output_file_handle:
    json.dump(head, output_file_handle, indent=0)
