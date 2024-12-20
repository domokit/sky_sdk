#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Copy a Dart package into a directory suitable for release."""

import argparse
import os
import shutil
import sys


def main():
  parser = argparse.ArgumentParser(description='Copy a Dart package')

  parser.add_argument('--source', type=str, help='Source directory assembled by dart_pkg.py')
  parser.add_argument('--dest', type=str, help='Destination directory for the package')

  args = parser.parse_args()

  if os.path.exists(args.dest):
    shutil.rmtree(args.dest)

  # dart_pkg.py will create a packages directory within the package.
  # Do not copy this into the release output.
  shutil.copytree(args.source, args.dest, ignore=shutil.ignore_patterns('packages'))


if __name__ == '__main__':
  sys.exit(main())
