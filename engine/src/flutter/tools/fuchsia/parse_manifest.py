#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

""" Parses manifest file and dumps it to json.
"""

import argparse
import json
import os
import sys
import hashlib


def main():
  parser = argparse.ArgumentParser()

  parser.add_argument('--input', dest='file_path', action='store', required=True)
  parser.add_argument('--clang-cpu', dest='clang_cpu', action='store', required=True)

  args = parser.parse_args()

  with open(args.file_path) as f:
    data = json.load(f)

  output = {}
  target = args.clang_cpu + '-fuchsia'

  for d in data:
    if target in d['target']:
      for runtime in d['runtime']:
        # key contains the soname and the cflags used to compile it.
        # this allows us to distinguish between different sanitizers
        # and experiments
        key = runtime['soname'] + ''.join(d['cflags'])
        md5 = hashlib.md5(key.encode()).hexdigest()
        hash_key = 'md5_%s' % md5
        # Uncomment this line to get the hash keys
        # print runtime['dist'], d['cflags'], hash_key
        output[hash_key] = os.path.dirname(runtime['dist'])

  print(json.dumps(output))

  return 0


if __name__ == '__main__':
  sys.exit(main())
