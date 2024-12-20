#!/usr/bin/env python3
#
# Copyright (c) 2012 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Prints the lowest locally available SDK version greater than or equal to a
given minimum sdk version to standard output.

Usage:
  python find_sdk.py 10.6  # Ignores SDKs < 10.6
"""

import json
import os
import re
import subprocess
import sys

from optparse import OptionParser

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from pyutil.file_util import symlink

PREBUILTS = os.path.realpath(os.path.join(
  os.path.dirname(__file__), os.pardir, os.pardir, 'flutter', 'prebuilts',
))

def parse_version(version_str):
  """'10.6' => [10, 6]"""
  return [int(x) for x in re.findall(r'(\d+)', version_str)]


def run_command_with_retry(command, timeout=10, retries=3):
  """
  Runs a command using subprocess.check_output with timeout and retry logic.

  Args:
      command: A list representing the command and its arguments.
      timeout: The maximum time (in seconds) to wait for each command execution.
      retries: The number of times to retry the command if it times out.

  Returns:
      The output of the command as a bytes object if successful, otherwise
      raises a CalledProcessError.
  """
  for attempt in range(1, retries + 1):
    try:
      result = subprocess.check_output(command, timeout=timeout)
      return result.decode('utf-8').strip()
    except subprocess.TimeoutExpired:
      if attempt >= retries:
        raise  # Re-raise the TimeoutExpired error after all retries


def main():
  parser = OptionParser()
  parser.add_option("--print_sdk_path",
                    action="store_true", dest="print_sdk_path", default=False,
                    help="Additionaly print the path the SDK (appears first).")
  parser.add_option("--as-gclient-hook",
                    action="store_true", dest="as_gclient_hook", default=False,
                    help="Whether the script is running as a gclient hook.")
  parser.add_option("--symlink",
                    action="store", type="string", dest="symlink",
                    help="Whether to create a symlink in the buildroot to the SDK.")
  (options, args) = parser.parse_args()
  min_sdk_version = args[0]

  # On CI, Xcode is not yet installed when gclient hooks are being run.
  # This is because the version of Xcode that CI installs might depend on the
  # contents of the repo, so the repo must be set up first, which includes
  # running the gclient hooks. Instead, on CI, this script will be run during
  # GN.
  running_on_luci = os.environ.get('LUCI_CONTEXT') is not None
  if running_on_luci and options.as_gclient_hook:
    return 0

  symlink_path = options.symlink
  if not running_on_luci and symlink_path is None:
    symlink_path = PREBUILTS

  job = subprocess.Popen(['xcode-select', '-print-path'],
                         universal_newlines=True,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT)
  out, err = job.communicate()
  if job.returncode != 0:
    sys.stderr.writelines([out, err])
    raise Exception(('Error %d running xcode-select, you might have to run '
      '|sudo xcode-select --switch /Applications/Xcode.app/Contents/Developer| '
      'if you are using Xcode 4.') % job.returncode)

  # xcrun --sdk macosx  --show-sdk-path
  sdk_command = [
    'xcrun',
    '--sdk',
    'macosx',
    '--show-sdk-path',
  ]
  sdk_output = run_command_with_retry(sdk_command, timeout=300)
  if symlink_path:
    sdks_path = os.path.join(symlink_path, 'SDKs')
    symlink_target = os.path.join(sdks_path, os.path.basename(sdk_output))
    symlink(sdk_output, symlink_target)
    sdk_output = symlink_target

  if not options.as_gclient_hook:
    print(sdk_output)
  return 0


if __name__ == '__main__':
  if sys.platform != 'darwin':
    raise Exception("This script only runs on Mac")
  sys.exit((main()))
