#!/usr/bin/env python3
# Copyright (c) 2013, the Flutter project authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file.

import os
import platform
import subprocess
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'test_scripts/test/')))

from common import catch_sigterm, wait_for_sigterm


def Main():
  """
    Executes the test-scripts with required environment variables. It acts like
    /usr/bin/env, but provides some extra functionality to dynamically set up
    the environment variables.
    """
  # Ensures the signals can be correctly forwarded to the subprocesses.
  catch_sigterm()

  os.environ['SRC_ROOT'] = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../'))
  # Flutter uses a different repo structure and fuchsia sdk is not in the
  # third_party/, so images root and sdk root need to be explicitly set.
  os.environ['FUCHSIA_IMAGES_ROOT'] = os.path.join(os.environ['SRC_ROOT'], 'fuchsia/images/')

  assert platform.system() == 'Linux', 'Unsupported OS ' + platform.system()
  os.environ['FUCHSIA_SDK_ROOT'] = os.path.join(os.environ['SRC_ROOT'], 'fuchsia/sdk/linux/')
  os.environ['FUCHSIA_GN_SDK_ROOT'] = os.path.join(
      os.environ['SRC_ROOT'], 'flutter/tools/fuchsia/gn-sdk/src'
  )

  if os.getenv('DOWNLOAD_FUCHSIA_SDK') == 'True':
    sdk_path = os.environ['FUCHSIA_SDK_PATH']
    assert sdk_path.endswith('/linux-amd64/core.tar.gz')
    assert not sdk_path.startswith('/')
    os.environ['FUCHSIA_SDK_OVERRIDE'
              ] = 'gs://fuchsia-artifacts/' + sdk_path[:-len('/linux-amd64/core.tar.gz')]

  with subprocess.Popen(sys.argv[1:]) as proc:
    try:
      proc.wait()
    except:
      # Use terminate / SIGTERM to allow the subprocess exiting cleanly.
      proc.terminate()
    return proc.returncode


if __name__ == '__main__':
  sys.exit(Main())
