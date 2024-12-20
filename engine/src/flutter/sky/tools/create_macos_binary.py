#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import os
import subprocess
import sys


def canonical_path(path):
  """Returns the canonical path for the input path.
  If the input path is not absolute, it is treated as relative to the engine
  source tree's buildroot directory."""
  if os.path.isabs(path):
    return path
  buildroot_dir = os.path.abspath(os.path.join(os.path.realpath(__file__), '..', '..', '..', '..'))
  return os.path.join(buildroot_dir, path)


def assert_file_exists(binary_path, arch):
  if not os.path.isfile(binary_path):
    print('Cannot find macOS %s binary at %s' % (arch, binary_path))
    sys.exit(1)


def create_universal_binary(in_arm64, in_x64, out):
  subprocess.check_call(['lipo', in_arm64, in_x64, '-create', '-output', out])


def main():
  parser = argparse.ArgumentParser(
      description='Creates a universal binary from input arm64, x64 binaries'
  )
  parser.add_argument('--in-arm64', type=str, required=True)
  parser.add_argument('--in-x64', type=str, required=True)
  parser.add_argument('--out', type=str, required=True)
  args = parser.parse_args()

  in_arm64 = canonical_path(args.in_arm64)
  in_x64 = canonical_path(args.in_x64)
  out = canonical_path(args.out)

  assert_file_exists(in_arm64, 'arm64')
  assert_file_exists(in_x64, 'x64')
  create_universal_binary(in_arm64, in_x64, out)

  return 0


if __name__ == '__main__':
  sys.exit(main())
