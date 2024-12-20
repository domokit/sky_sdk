# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Helper script for GN to run malioc.

This is the same as `gn_run_binary.py`, except an extra parameter is included
for the malioc output file. When the malioc run fails, errors are placed in the
json output file. This script attempts to read the output file and dump it to
stdout upon failure.

Run with:
  python gn_run_malioc.py <binary_name> <output_path> [args ...]
"""

import json
import os
import sys
import subprocess

# This script is designed to run binaries produced by the current build. We
# always prefix it with "./" to avoid picking up system versions that might
# also be on the path.
path = './' + sys.argv[1]

malioc_output = sys.argv[2]

# The rest of the arguements are passed directly to the executable.
args = [path, '--output', malioc_output] + sys.argv[3:]

try:
  subprocess.check_output(args, stderr=subprocess.STDOUT)
except subprocess.CalledProcessError as ex:
  print(ex.output.decode('utf-8', errors='replace'))
  if os.path.exists(malioc_output):
    with open(malioc_output, 'r') as malioc_file:
      malioc_json = malioc_file.read()

    print('malioc output:')
    # Attempt to pretty print the json output, but fall back to printing the
    # raw output if doing so fails.
    try:
      parsed = json.loads(malioc_json)
      print(json.dumps(parsed, indent=2))
    except:
      print(malioc_json)

  else:
    print(
        'Unable to find the malioc output file in order to print contained'
        'errors:',
        malioc_output,
    )
  sys.exit(ex.returncode)
