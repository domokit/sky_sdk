#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

""" Interpolates test suite information into a cml file.
"""

from argparse import ArgumentParser
import sys


def main():
  # Parse arguments.
  parser = ArgumentParser()
  parser.add_argument('--input', action='store', required=True)
  parser.add_argument('--test-suite', action='store', required=True)
  parser.add_argument('--output', action='store', required=True)
  args = parser.parse_args()

  # Read, interpolate, write.
  with open(args.input, 'r') as i, open(args.output, 'w') as o:
    o.write(i.read().replace('{{TEST_SUITE}}', args.test_suite))

  return 0


if __name__ == '__main__':
  sys.exit(main())
