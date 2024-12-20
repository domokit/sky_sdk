#!/usr/bin/env python3
#
# Copyright (c) 2011 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Rewrites paths in -I, -L and other option to be relative to a sysroot."""

import sys
import os
import optparse

REWRITE_PREFIX = ['-I',
                  '-idirafter',
                  '-imacros',
                  '-imultilib',
                  '-include',
                  '-iprefix',
                  '-iquote',
                  '-isystem',
                  '-L']

def RewritePath(path, opts):
  """Rewrites a path by stripping the prefix and prepending the sysroot."""
  sysroot = opts.sysroot
  prefix = opts.strip_prefix
  if os.path.isabs(path) and not path.startswith(sysroot):
    if path.startswith(prefix):
      path = path[len(prefix):]
    path = path.lstrip('/')
    return os.path.join(sysroot, path)
  else:
    return path


def RewriteLine(line, opts):
  """Rewrites all the paths in recognized options."""
  args = line.split()
  count = len(args)
  i = 0
  while i < count:
    for prefix in REWRITE_PREFIX:
      # The option can be either in the form "-I /path/to/dir" or
      # "-I/path/to/dir" so handle both.
      if args[i] == prefix:
        i += 1
        try:
          args[i] = RewritePath(args[i], opts)
        except IndexError:
          sys.stderr.write('Missing argument following %s\n' % prefix)
          break
      elif args[i].startswith(prefix):
        args[i] = prefix + RewritePath(args[i][len(prefix):], opts)
    i += 1

  return ' '.join(args)


def main(argv):
  parser = optparse.OptionParser()
  parser.add_option('-s', '--sysroot', default='/', help='sysroot to prepend')
  parser.add_option('-p', '--strip-prefix', default='', help='prefix to strip')
  opts, args = parser.parse_args(argv[1:])

  for line in sys.stdin.readlines():
    line = RewriteLine(line.strip(), opts)
    print(line)
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv))
