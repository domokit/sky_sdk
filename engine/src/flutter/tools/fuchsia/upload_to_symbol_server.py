#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Uploads debug symbols to the symbols server."""

import argparse
import os
import subprocess
import sys
import tempfile

## Path to the engine root checkout. This is used to calculate absolute
## paths if relative ones are passed to the script.
BUILD_ROOT_DIR = os.path.abspath(os.path.join(os.path.realpath(__file__), '..', '..', '..', '..'))
FUCHSIA_ARTIFACTS_DEBUG_NAMESPACE = 'debug'
FUCHSIA_ARTIFACTS_BUCKET_NAME = 'fuchsia-artifacts-release'


def remote_filename(exec_path):
  # An example of exec_path is:
  # out/fuchsia_debug_x64/flutter-fuchsia-x64/d4/917f5976.debug
  # In the above example "d4917f5976" is the elf BuildID for the
  # executable. First 2 characters are used as the directory name
  # and the rest of the string is the name of the unstripped executable.
  parts = exec_path.split('/')
  # We want d4917f5976.debug as the result.
  return ''.join(parts[-2:])


def exists_remotely(remote_path):
  gsutil = os.path.join(os.environ['DEPOT_TOOLS'], 'gsutil.py')
  command = ['python3', gsutil, '--', 'stat', remote_path]
  process = subprocess.Popen(command, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
  stdout, stderr = process.communicate()
  return_code = process.wait()
  if return_code == 0:
    print('%s exists - skipping copy' % remote_path)
  return return_code == 0


def process_symbols(should_upload, symbol_dir):
  full_path = os.path.join(BUILD_ROOT_DIR, symbol_dir)

  files = []
  for (dirpath, dirnames, filenames) in os.walk(full_path):
    files.extend([os.path.join(dirpath, f) for f in filenames])

  print('List of files to upload')
  print('\n'.join(files))

  # Remove dbg_files
  files = [f for f in files if 'dbg_success' not in f]

  for file in files:
    remote_path = 'gs://%s/%s/%s' % (
        FUCHSIA_ARTIFACTS_BUCKET_NAME, FUCHSIA_ARTIFACTS_DEBUG_NAMESPACE, remote_filename(file)
    )
    if should_upload and not exists_remotely(remote_path):
      gsutil = os.path.join(os.environ['DEPOT_TOOLS'], 'gsutil.py')
      command = ['python3', gsutil, '--', 'cp', file, remote_path]
      subprocess.check_call(command)
    else:
      print(remote_path)


def main():
  parser = argparse.ArgumentParser()

  parser.add_argument(
      '--symbol-dir', required=True, help='Directory that contain the debug symbols.'
  )
  parser.add_argument('--engine-version', required=True, help='Specifies the flutter engine SHA.')
  parser.add_argument(
      '--upload', default=False, action='store_true', help='If set, uploads symbols to the server.'
  )

  args = parser.parse_args()

  should_upload = args.upload
  engine_version = args.engine_version
  if not engine_version:
    engine_version = 'HEAD'
    should_upload = False

  process_symbols(should_upload, args.symbol_dir)
  return 0


if __name__ == '__main__':
  sys.exit(main())
