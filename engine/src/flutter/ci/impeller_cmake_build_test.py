#!/usr/bin/env vpython3
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import os
import subprocess
import sys

# When passed the --setup flag, this script fetches git submodules and other
# dependencies for the impeller-cmake-example. When passed the --cmake flag,
# this script runs cmake on impeller-cmake-example. That will create
# a build output directory for impeller-cmake-example under
# out/impeller-cmake-example, so the build can then be performed with
# e.g. ninja -C out/impeller-cmake-example-out.

SRC_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def parse_args(argv):
  parser = argparse.ArgumentParser(
      description='A script that tests the impeller-cmake-example build.',
  )
  parser.add_argument(
      '--cmake',
      '-c',
      default=False,
      action='store_true',
      help='Run cmake for impeller-cmake-example.',
  )
  parser.add_argument(
      '--path',
      '-p',
      type=str,
      help='The path to the impeller-cmake-example source.',
  )
  parser.add_argument(
      '--setup',
      '-s',
      default=False,
      action='store_true',
      help='Clone the git submodules.',
  )
  parser.add_argument(
      '--verbose',
      '-v',
      default=False,
      action='store_true',
      help='Emit verbose output.',
  )
  parser.add_argument(
      '--xcode-symlinks',
      default=False,
      action='store_true',
      help='Symlink the Xcode sysroot to help Goma be successful.',
  )
  return parser.parse_args(argv)


def validate_args(args):
  if not os.path.isdir(os.path.join(SRC_ROOT, args.path)):
    print(
        'The --path argument must be a valid directory relative to the '
        'engine src/ directory.'
    )
    return False

  return True


def create_xcode_symlink():
  find_sdk_command = [
      'python3',
      os.path.join(SRC_ROOT, 'build', 'mac', 'find_sdk.py'),
      '--print_sdk_path',
      '10.15',
      '--symlink',
      os.path.join(SRC_ROOT, 'out', 'impeller-cmake-example-xcode-sysroot'),
  ]
  find_sdk_output = subprocess.check_output(find_sdk_command).decode('utf-8')
  return find_sdk_output.split('\n')[0]


def main(argv):
  args = parse_args(argv[1:])
  if not validate_args(args):
    return 1

  impeller_cmake_dir = os.path.join(SRC_ROOT, args.path)

  if args.setup:
    git_command = [
        'git',
        '-C',
        impeller_cmake_dir,
        'submodule',
        'update',
        '--init',
        '--recursive',
        '--depth',
        '1',
        '--jobs',
        str(os.cpu_count()),
    ]
    subprocess.check_call(git_command)

    # Run the deps.sh shell script in the repo.
    subprocess.check_call(['bash', 'deps.sh'], cwd=impeller_cmake_dir)
    return 0

  if args.cmake:
    cmake_path = os.path.join(SRC_ROOT, 'buildtools', 'mac-x64', 'cmake', 'bin', 'cmake')
    cmake_command = [
        cmake_path,
        '--preset',
        'flutter-ci-mac-debug-x64',
        '-B',
        os.path.join(SRC_ROOT, 'out', 'impeller-cmake-example'),
    ]
    cmake_env = os.environ.copy()
    ninja_path = os.path.join(SRC_ROOT, 'flutter', 'third_party', 'ninja')
    cmake_env.update({
        'PATH': os.environ['PATH'] + ':' + ninja_path,
        'FLUTTER_ENGINE_SRC_DIR': SRC_ROOT,
    })
    if args.xcode_symlinks:
      xcode_symlink_path = create_xcode_symlink()
      cmake_env.update({
          'FLUTTER_OSX_SYSROOT': xcode_symlink_path,
      })
    subprocess.check_call(cmake_command, env=cmake_env, cwd=impeller_cmake_dir)

  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv))
