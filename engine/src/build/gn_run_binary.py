# Copyright 2014 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Helper script for GN to run an arbitrary binary. See compiled_action.gni.

Run with:
  python gn_run_binary.py <binary_name> [args ...]
"""

import sys
import subprocess

# This script is designed to run binaries produced by the current build. We
# always prefix it with "./" to avoid picking up system versions that might
# also be on the path.
path = './' + sys.argv[1]

# The rest of the arguements are passed directly to the executable.
args = [path] + sys.argv[2:]

try:
  subprocess.check_output(args, stderr=subprocess.STDOUT)
except subprocess.CalledProcessError as ex:
  print("Command failed: " + ' '.join(args))
  print("exitCode: " + str(ex.returncode))
  print(ex.output.decode('utf-8', errors='replace'))
  sys.exit(ex.returncode)
