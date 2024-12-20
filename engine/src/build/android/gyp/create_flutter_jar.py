#!/usr/bin/env python3
#
# Copyright 2016 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Create a JAR incorporating all the components required to build a Flutter application"""

import optparse
import os
import sys
import zipfile

from util import build_utils

def main(args):
  args = build_utils.ExpandFileArgs(args)
  parser = optparse.OptionParser()
  build_utils.AddDepfileOption(parser)
  parser.add_option('--output', help='Path to output jar.')
  parser.add_option('--output_native_jar', help='Path to output native library jar.')
  parser.add_option('--dist_jar', help='Flutter shell Java code jar.')
  parser.add_option('--native_lib', action='append', help='Native code library.')
  parser.add_option('--android_abi', help='Native code ABI.')
  parser.add_option('--asset_dir', help='Path to assets.')
  options, _ = parser.parse_args(args)
  build_utils.CheckOptions(options, parser, [
    'output', 'dist_jar', 'native_lib', 'android_abi'
  ])

  input_deps = []

  with zipfile.ZipFile(options.output, 'w', zipfile.ZIP_DEFLATED) as out_zip:
    input_deps.append(options.dist_jar)
    with zipfile.ZipFile(options.dist_jar, 'r') as dist_zip:
      for dist_file in dist_zip.infolist():
        if dist_file.filename.endswith('.class'):
          out_zip.writestr(dist_file.filename, dist_zip.read(dist_file.filename))

    for native_lib in options.native_lib:
      input_deps.append(native_lib)
      out_zip.write(native_lib,
                    'lib/%s/%s' % (options.android_abi, os.path.basename(native_lib)))

    if options.asset_dir:
      for asset_file in os.listdir(options.asset_dir):
        input_deps.append(asset_file)
        out_zip.write(os.path.join(options.asset_dir, asset_file),
                      'assets/flutter_shared/%s' % asset_file)

  if options.output_native_jar:
    with zipfile.ZipFile(options.output_native_jar, 'w', zipfile.ZIP_DEFLATED) as out_zip:
      for native_lib in options.native_lib:
        out_zip.write(native_lib,
                      'lib/%s/%s' % (options.android_abi, os.path.basename(native_lib)))

  if options.depfile:
    build_utils.WriteDepfile(
        options.depfile,
        input_deps + build_utils.GetPythonDependencies())


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
