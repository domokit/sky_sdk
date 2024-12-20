#!/usr/bin/env python3
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Emulation of `rm -f out && cp -af` in out. This is necessary on Mac in order
to preserve nanoseconds of mtime. See https://fxbug.dev/56376#c5."""

import os
import shutil
import sys


def main():
  if len(sys.argv) != 3:
    print('usage: copy.py source dest', file=sys.stderr)
    return 1
  source = sys.argv[1]
  dest = sys.argv[2]

  if os.path.isdir(source):
    print(f'{source} is a directory, tool "copy" does not support directory copies')
    return 1

  if os.path.exists(dest):
    if os.path.isdir(dest):

      def _on_error(fn, path, dummy_excinfo):
        # The operation failed, possibly because the file is set to
        # read-only. If that's why, make it writable and try the op
        # again.
        if not os.access(path, os.W_OK):
          os.chmod(path, stat.S_IWRITE)
        fn(path)

      shutil.rmtree(dest, onerror=_on_error)
    else:
      if not os.access(dest, os.W_OK):
        # Attempt to make the file writable before deleting it.
        os.chmod(dest, stat.S_IWRITE)
      os.unlink(dest)

  shutil.copy2(source, dest)


if __name__ == '__main__':
  main()
