#!/usr/bin/env python3

"""Merges sources of a Dart target and its dependencies"""

# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import json
import os
import sys


def main():
  parser = argparse.ArgumentParser(
      'Merges sources of a Dart target and its dependencies', fromfile_prefix_chars='@'
  )
  parser.add_argument(
      '--output', help='Path to output the final list', type=argparse.FileType('w'), required=True
  )
  parser.add_argument(
      '--depfile',
      help='Path to the depfile to generate',
      type=argparse.FileType('w'),
      required=True
  )
  parser.add_argument(
      '--sources',
      help='Sources of this target',
      nargs='*',
  )
  parser.add_argument('--source_lists', help='Files containing lists of Dart sources', nargs='*')
  args = parser.parse_args()

  args.depfile.write('{}: {}\n'.format(args.output.name, ' '.join(args.source_lists)))

  # Merges sources of this target, and all of its dependencies.
  all_sources = set(args.sources)
  for f in args.source_lists:
    with open(f, 'r') as f:
      all_sources.update(json.load(f))
  json.dump(sorted(all_sources), args.output)


if __name__ == '__main__':
  sys.exit(main())
