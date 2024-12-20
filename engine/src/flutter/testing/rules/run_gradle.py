#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""
Invokes //gradle for building the Android apps from GN/Ninja.
"""

import os
import sys
import subprocess
import platform

SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__))

BAT = '.bat' if sys.platform.startswith(('cygwin', 'win')) else ''
GRADLE_BIN = os.path.normpath(
    os.path.join(SCRIPT_PATH, '..', '..', 'third_party', 'gradle', 'bin', 'gradle%s' % BAT)
)

ANDROID_HOME = os.path.normpath(
    os.path.join(SCRIPT_PATH, '..', '..', 'third_party', 'android_tools', 'sdk')
)

if platform.system() == 'Darwin':
  JAVA_HOME = os.path.normpath(
      os.path.join(SCRIPT_PATH, '..', '..', 'third_party', 'java', 'openjdk', 'Contents', 'Home')
  )
else:
  JAVA_HOME = os.path.normpath(
      os.path.join(SCRIPT_PATH, '..', '..', 'third_party', 'java', 'openjdk')
  )


def main():
  if not os.path.isdir(ANDROID_HOME):
    raise Exception('%s (ANDROID_HOME) is not a directory' % ANDROID_HOME)

  android_dir = sys.argv[1]
  subprocess.check_output(
      args=[GRADLE_BIN] + sys.argv[2:],
      cwd=android_dir,
      env=dict(os.environ, ANDROID_HOME=ANDROID_HOME, JAVA_HOME=JAVA_HOME),
  )
  return 0


if __name__ == '__main__':
  sys.exit(main())
