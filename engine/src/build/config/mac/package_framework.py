# Copyright 2016 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import errno
import os
import shutil
import sys

def Main():
  parser = argparse.ArgumentParser(description='Create Mac Framework symlinks')
  parser.add_argument('--framework', action='store', type=str, required=True)
  parser.add_argument('--version', action='store', type=str)
  parser.add_argument('--contents', action='store', type=str, nargs='+')
  parser.add_argument('--stamp', action='store', type=str, required=True)
  args = parser.parse_args()

  VERSIONS = 'Versions'
  CURRENT = 'Current'

  # Ensure the Foo.framework/Versions/A/ directory exists and create the
  # Foo.framework/Versions/Current symlink to it.
  if args.version:
    try:
      os.makedirs(os.path.join(args.framework, VERSIONS, args.version), 744)
    except OSError as e:
      if e.errno != errno.EEXIST:
        raise e
    _Relink(os.path.join(args.version),
            os.path.join(args.framework, VERSIONS, CURRENT))

  # Establish the top-level symlinks in the framework bundle. The dest of
  # the symlinks may not exist yet.
  if args.contents:
    for item in args.contents:
      _Relink(os.path.join(VERSIONS, CURRENT, item),
              os.path.join(args.framework, item))

  # Write out a stamp file.
  if args.stamp:
    with open(args.stamp, 'w') as f:
      f.write(str(args))

  return 0


def _Relink(dest, link):
  """Creates a symlink to |dest| named |link|. If |link| already exists,
  it is overwritten."""
  try:
    os.remove(link)
  except OSError as e:
    if e.errno != errno.ENOENT:
      shutil.rmtree(link)
  os.symlink(dest, link)


if __name__ == '__main__':
  sys.exit(Main())
