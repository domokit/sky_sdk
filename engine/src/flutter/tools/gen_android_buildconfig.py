#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import os
import sys

BUILD_CONFIG_TEMPLATE = """
// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.


// THIS FILE IS AUTO_GENERATED
// DO NOT EDIT THE VALUES HERE - SEE $flutter_root/tools/gen_android_buildconfig.py
package io.flutter;

public final class BuildConfig {{
  private BuildConfig() {{}}

  public final static boolean DEBUG = {0};
  public final static boolean PROFILE = {1};
  public final static boolean RELEASE = {2};
  public final static boolean JIT_RELEASE = {3};
}}
"""


def main():
  parser = argparse.ArgumentParser(description='Generate BuildConfig.java for Android')
  parser.add_argument('--runtime-mode', type=str, required=True)
  parser.add_argument('--out', type=str, required=True)

  args = parser.parse_args()

  jit_release = 'jit_release' in args.runtime_mode.lower()
  release = not jit_release and 'release' in args.runtime_mode.lower()
  profile = 'profile' in args.runtime_mode.lower()
  debug = 'debug' in args.runtime_mode.lower()
  assert debug or profile or release or jit_release

  with open(os.path.abspath(args.out), 'w+') as output_file:
    output_file.write(
        BUILD_CONFIG_TEMPLATE.format(
            str(debug).lower(),
            str(profile).lower(),
            str(release).lower(),
            str(jit_release).lower()
        )
    )


if __name__ == '__main__':
  sys.exit(main())
