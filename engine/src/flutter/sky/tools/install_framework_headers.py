#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import errno
import os
import shutil
import sys


def main():
  parser = argparse.ArgumentParser(
      description='Removes existing files and installs the specified headers' +
      'at the given location.'
  )

  parser.add_argument(
      '--headers', nargs='+', help='The headers to install at the location.', required=True
  )
  parser.add_argument('--location', type=str, required=True)

  args = parser.parse_args()

  # Remove old headers.
  try:
    shutil.rmtree(os.path.normpath(args.location))
  except OSError as err:
    # Ignore only "not found" errors.
    if err.errno != errno.ENOENT:
      raise err

  # Create the directory to copy the files to.
  if not os.path.isdir(args.location):
    os.makedirs(args.location)

  # Copy all files specified in the args.
  for header_file in args.headers:
    shutil.copyfile(header_file, os.path.join(args.location, os.path.basename(header_file)))


if __name__ == '__main__':
  sys.exit(main())
