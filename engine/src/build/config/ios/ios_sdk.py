# Copyright 2014 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import errno
import os
import shutil
import subprocess
import sys

sys.path.insert(1, os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))
from pyutil.file_util import symlink

# This script creates symlinks under flutter/prebuilts to the iphone and
# iphone simulator SDKs.

SDKs = ['iphoneos', 'iphonesimulator']

PREBUILTS = os.path.realpath(os.path.join(
  os.path.dirname(__file__), os.pardir, os.pardir, os.pardir, 'flutter', 'prebuilts',
))


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


def main(argv):
  parser = argparse.ArgumentParser()
  parser.add_argument(
      '--as-gclient-hook',
      default=False,
      action='store_true',
      help='Whether the script is running as a gclient hook.',
  )
  parser.add_argument(
    '--symlink',
    type=str,
    help='Whether to create a symlink in the buildroot to the SDK.',
  )
  parser.add_argument(
    '--sdk',
    choices=['iphoneos', 'iphonesimulator'],
    help='Which SDK to find.',
  )
  args = parser.parse_args()

  # On CI, Xcode is not yet installed when gclient hooks are being run.
  # This is because the version of Xcode that CI installs might depend on the
  # contents of the repo, so the repo must be set up first, which includes
  # running the gclient hooks. Instead, on CI, this script will be run during
  # GN.
  running_on_luci = os.environ.get('LUCI_CONTEXT') is not None
  if running_on_luci and args.as_gclient_hook:
    return 0

  symlink_path = args.symlink
  if not running_on_luci and symlink_path is None:
    symlink_path = PREBUILTS

  sdks = [args.sdk] if args.sdk is not None else SDKs

  sdks_path = None
  libraries_path = None
  if symlink_path:
    sdks_path = os.path.join(symlink_path, 'SDKs')
    libraries_path = os.path.join(symlink_path, 'Library')
    # Remove any old files created by this script under PREBUILTS/SDKs.
    if args.as_gclient_hook:
      if os.path.isdir(sdks_path):
        shutil.rmtree(sdks_path)
      if os.path.isdir(libraries_path):
        shutil.rmtree(libraries_path)

  for sdk in sdks:
    command =  [
      'xcrun',
      '--sdk',
      sdk,
      '--show-sdk-path',
    ]
    sdk_output = run_command_with_retry(command, timeout=300)
    if symlink_path:
      symlink_target = os.path.join(sdks_path, os.path.basename(sdk_output))
      symlink(sdk_output, symlink_target)
      frameworks_location = os.path.join(sdk_output, '..', '..', 'Library', 'Frameworks')
      frameworks_symlink = os.path.join(libraries_path, 'Frameworks')
      symlink(frameworks_location, frameworks_symlink)
      sdk_output = symlink_target
    if not args.as_gclient_hook:
      print(sdk_output)
  return 0


if __name__ == '__main__':
  if sys.platform != 'darwin':
    raise Exception('This script only runs on Mac')
  sys.exit(main(sys.argv))
