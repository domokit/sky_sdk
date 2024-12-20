#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Copies and renames android artifacts."""

import argparse
import os
import shutil
import sys


def cp_files(args):
  """Copies files from source to destination.

  It creates the destination folder if it does not exists yet.
  """
  for src, dst in args.input_pairs:
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copyfile(src, dst)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument(
      '-i',
      dest='input_pairs',
      nargs=2,
      action='append',
      help='The input file and its destination.'
  )
  cp_files(parser.parse_args())
  return 0


if __name__ == '__main__':
  sys.exit(main())
