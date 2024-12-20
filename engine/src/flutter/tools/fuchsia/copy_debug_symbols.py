#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

""" Gather the build_id, prefix_dir, and exec_name given the path to executable
    also copies to the specified destination.

    The structure of debug symbols is as follows:
      .build-id/<prefix>/<exec_name>[.debug]
"""

import argparse
import errno
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time


def HashFile(filepath):
  """Calculates the hash of a file without reading it all in memory at once."""
  digest = hashlib.sha1()
  with open(filepath, 'rb') as f:
    while True:
      chunk = f.read(1024 * 1024)
      if not chunk:
        break
      digest.update(chunk)
  return digest.hexdigest()


def Touch(fname):
  with open(fname, 'a'):
    os.utime(fname, None)


def GetBuildIdParts(exec_path, read_elf):
  sha1_pattern = re.compile(r'[0-9a-fA-F\-]+')
  file_out = subprocess.check_output([read_elf, '-n', exec_path])
  build_id_line = file_out.splitlines()[-1].split()
  if (build_id_line[0] != b'Build' or build_id_line[1] != b'ID:' or
      not sha1_pattern.match(str(build_id_line[-1])) or not len(build_id_line[-1]) > 2):
    raise Exception(
        'Expected the last line of llvm-readelf to match "Build ID <Hex String>" Got: %s' % file_out
    )

  build_id = build_id_line[-1]
  return {
      'build_id': build_id.decode('utf-8'), 'prefix_dir': build_id[:2].decode('utf-8'),
      'exec_name': build_id[2:].decode('utf-8')
  }


def main():
  parser = argparse.ArgumentParser()

  parser.add_argument(
      '--executable-name',
      dest='exec_name',
      action='store',
      required=True,
      help='This is the name of the executable that we wish to layout debug symbols for.'
  )
  parser.add_argument(
      '--executable-path',
      dest='exec_path',
      action='store',
      required=True,
      help='Path to the executable on the filesystem.'
  )
  parser.add_argument(
      '--destination-base',
      dest='dest',
      action='store',
      required=True,
      help='Path to the base directory where the debug symbols are to be laid out.'
  )
  parser.add_argument(
      '--stripped',
      dest='stripped',
      action='store_true',
      default=True,
      help='Executable at the specified path is stripped.'
  )
  parser.add_argument(
      '--unstripped',
      dest='stripped',
      action='store_false',
      help='Executable at the specified path is unstripped.'
  )
  parser.add_argument(
      '--read-elf',
      dest='read_elf',
      action='store',
      required=True,
      help='Path to read-elf executable.'
  )

  args = parser.parse_args()
  assert os.path.exists(args.exec_path), ('exec_path "%s" does not exist' % args.exec_path)
  assert os.path.exists(args.dest), ('dest "%s" does not exist' % args.dest)
  assert os.path.exists(args.read_elf), ('read_elf "%s" does not exist' % args.read_elf)

  parts = GetBuildIdParts(args.exec_path, args.read_elf)
  dbg_prefix_base = os.path.join(args.dest, parts['prefix_dir'])

  # Multiple processes may be trying to create the same directory.
  # TODO(dnfield): use exist_ok when we upgrade to python 3, rather than try
  try:
    os.makedirs(dbg_prefix_base)
  except OSError as e:
    if e.errno != errno.EEXIST:
      raise

  if not os.path.exists(dbg_prefix_base):
    print('Unable to create directory: %s.' % dbg_prefix_base)
    return 1

  dbg_suffix = ''
  if not args.stripped:
    dbg_suffix = '.debug'
  dbg_file_name = '%s%s' % (parts['exec_name'], dbg_suffix)
  dbg_file_path = os.path.join(dbg_prefix_base, dbg_file_name)

  # If the debug file hasn't changed, don't rewrite the debug and completion
  # file, speeding up incremental builds.
  if os.path.exists(dbg_file_path) and HashFile(args.exec_path) == HashFile(dbg_file_path):
    return 0

  shutil.copyfile(args.exec_path, dbg_file_path)

  # Note this needs to be in sync with fuchsia_debug_symbols.gni
  completion_file = os.path.join(args.dest, '.%s_dbg_success' % args.exec_name)
  Touch(completion_file)

  return 0


if __name__ == '__main__':
  sys.exit(main())
