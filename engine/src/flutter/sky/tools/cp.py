#!/usr/bin/env python3
#
# Copyright (c) 2012 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Copy a file.

This module works much like the cp posix command - it takes 2 arguments:
(src, dst) and copies the file with path |src| to |dst|.
"""

import os
import shutil
import sys


def main(src, dst):
  # Use copy instead of copyfile to ensure the executable bit is copied.
  dstpath = os.path.normpath(dst)
  try:
    shutil.copy(src, dstpath)
  except shutil.SameFileError:
    if not (os.path.islink(dstpath) or os.stat(dstpath).st_nlink > 1):
      raise
    # Copy will fail if the destination is the link to the source.
    # If that's the case, then delete the destination link first,
    # then repeat the copy.
    os.remove(dstpath)
    shutil.copy(src, dstpath)
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1], sys.argv[2]))
