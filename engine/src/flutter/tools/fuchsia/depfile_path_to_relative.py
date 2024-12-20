#!/usr/bin/env python3
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import os
import subprocess
import sys


def main():
  parser = argparse.ArgumentParser(
      description='Executes a command, then rewrites the depfile, converts all absolute paths to relative'
  )
  parser.add_argument('--depfile', help='Path to the depfile to rewrite', required=True)
  parser.add_argument('command', nargs='+', help='Positional args for the command to run')
  args = parser.parse_args()

  retval = subprocess.call(args.command)
  if retval != 0:
    return retval

  lines = []
  with open(args.depfile, 'r') as f:
    for line in f:
      lines.append(' '.join(os.path.relpath(p) for p in line.split()))
  with open(args.depfile, 'w') as f:
    f.write('\n'.join(lines))


if __name__ == '__main__':
  sys.exit(main())
