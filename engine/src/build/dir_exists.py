#!/usr/bin/env python3
#
# Copyright (c) 2011 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
"""Writes True if the argument is a directory."""

import os.path
import sys

def main():
  sys.stdout.write(_is_dir(sys.argv[1]))
  return 0

def _is_dir(dir_name):
  return str(os.path.isdir(dir_name))

def DoMain(args):
  """Hook to be called from gyp without starting a separate python
  interpreter."""
  return _is_dir(args[0])

if __name__ == '__main__':
  sys.exit(main())
