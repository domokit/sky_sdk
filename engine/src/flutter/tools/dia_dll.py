#!/usr/bin/env python3
#
# Copyright (c) 2012 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""
This script is based on chromium/chromium/main/tools/clang/scripts/update.py.

It is used on Windows platforms to copy the correct msdia*.dll to the
clang folder, as a "gclient hook".
"""

import os
import shutil
import stat
import sys

# Path constants. (All of these should be absolute paths.)
THIS_DIR = os.path.abspath(os.path.dirname(__file__))
LLVM_BUILD_DIR = os.path.abspath(os.path.join(THIS_DIR, '..', 'buildtools', 'windows-x64', 'clang'))


def GetDiaDll():
  """Get the location of msdia*.dll for the platform."""

  # Bump after VC updates.
  DIA_DLL = {
      '2013': 'msdia120.dll',
      '2015': 'msdia140.dll',
      '2017': 'msdia140.dll',
      '2019': 'msdia140.dll',
      '2022': 'msdia140.dll',
  }

  # Don't let vs_toolchain overwrite our environment.
  environ_bak = os.environ

  sys.path.append(os.path.join(THIS_DIR, '..', '..', 'build'))
  import vs_toolchain
  win_sdk_dir = vs_toolchain.SetEnvironmentAndGetSDKDir()
  msvs_version = vs_toolchain.GetVisualStudioVersion()

  if bool(int(os.environ.get('DEPOT_TOOLS_WIN_TOOLCHAIN', '1'))):
    # Depot tools .zip layout:
    # \- DIA SDK
    # \- Windows Kits
    #   \- 10
    #     \- ... # Location of `DEPOT_TOOLS_WIN_TOOLCHAIN`
    dia_path = os.path.join(win_sdk_dir, '..', '..', 'DIA SDK', 'bin', 'amd64')
  else:
    if 'GYP_MSVS_OVERRIDE_PATH' in os.environ:
      vs_path = os.environ['GYP_MSVS_OVERRIDE_PATH']
    else:
      vs_path = vs_toolchain.DetectVisualStudioPath()
    dia_path = os.path.join(vs_path, 'DIA SDK', 'bin', 'amd64')

  os.environ = environ_bak
  return os.path.join(dia_path, DIA_DLL[msvs_version])


def CopyFile(src, dst):
  """Copy a file from src to dst."""
  print("Copying %s to %s" % (str(src), str(dst)))
  shutil.copy(src, dst)


def CopyDiaDllTo(target_dir):
  # This script always wants to use the 64-bit msdia*.dll.
  dia_dll = GetDiaDll()
  CopyFile(dia_dll, target_dir)


def main():
  CopyDiaDllTo(os.path.join(LLVM_BUILD_DIR, 'bin'))
  return 0


if __name__ == '__main__':
  sys.exit(main())
