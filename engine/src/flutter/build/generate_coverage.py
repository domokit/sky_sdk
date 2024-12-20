#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import sys
import subprocess
import os
import argparse
import errno
import shutil


def get_llvm_bin_directory():
  buildtool_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../buildtools')
  platform_dir = ''
  if sys.platform.startswith('linux'):
    platform_dir = 'linux-x64'
  elif sys.platform == 'darwin':
    platform_dir = 'mac-x64'
  else:
    raise Exception('Unknown/Unsupported platform.')
  llvm_bin_dir = os.path.abspath(os.path.join(buildtool_dir, platform_dir, 'clang/bin'))
  if not os.path.exists(llvm_bin_dir):
    raise Exception('LLVM directory %s double not be located.' % llvm_bin_dir)
  return llvm_bin_dir


def make_dirs(new_dir):
  """A wrapper around os.makedirs() that emulates "mkdir -p"."""
  try:
    os.makedirs(new_dir)
  except OSError as err:
    if err.errno != errno.EEXIST:
      raise


def remove_if_exists(path):
  if os.path.isdir(path) and not os.path.islink(path):
    shutil.rmtree(path)
  elif os.path.exists(path):
    os.remove(path)


def collect_profiles(args):
  raw_profiles = []
  binaries = []

  # Run all unit tests and collect raw profiles.
  for test in args.tests:
    absolute_test_path = os.path.abspath(test)
    absolute_test_dir = os.path.dirname(absolute_test_path)
    test_name = os.path.basename(absolute_test_path)

    if not os.path.exists(absolute_test_path):
      print('Path %s does not exist.' % absolute_test_path)
      return -1

    unstripped_test_path = os.path.join(absolute_test_dir, 'exe.unstripped', test_name)

    if os.path.exists(unstripped_test_path):
      binaries.append(unstripped_test_path)
    else:
      binaries.append(absolute_test_path)

    raw_profile = absolute_test_path + '.rawprofile'

    remove_if_exists(raw_profile)

    print('Running test %s to gather profile.' % os.path.basename(absolute_test_path))

    test_command = [absolute_test_path]

    test_args = ' '.join(args.test_args).split()

    if test_args is not None:
      test_command += test_args

    subprocess.check_call(test_command, env={'LLVM_PROFILE_FILE': raw_profile})

    if not os.path.exists(raw_profile):
      print('Could not find raw profile data for unit test run %s.' % test)
      print('Did you build with the --coverage flag?')
      return -1

    raw_profiles.append(raw_profile)

  return (binaries, raw_profiles)


def merge_profiles(llvm_bin_dir, raw_profiles, output):
  # Merge all raw profiles into a single profile.
  profdata_binary = os.path.join(llvm_bin_dir, 'llvm-profdata')

  print('Merging %d raw profile(s) into single profile.' % len(raw_profiles))
  merged_profile_path = os.path.join(output, 'all.profile')
  remove_if_exists(merged_profile_path)
  merge_command = [profdata_binary, 'merge', '-sparse'] + raw_profiles + ['-o', merged_profile_path]
  subprocess.check_call(merge_command)
  print('Done.')
  return merged_profile_path


def main():
  parser = argparse.ArgumentParser()

  parser.add_argument(
      '-t',
      '--tests',
      nargs='+',
      dest='tests',
      required=True,
      help='The unit tests to run and gather coverage data on.'
  )
  parser.add_argument(
      '-o',
      '--output',
      dest='output',
      required=True,
      help='The output directory for coverage results.'
  )
  parser.add_argument(
      '-f',
      '--format',
      type=str,
      choices=['all', 'html', 'summary', 'lcov'],
      required=True,
      help='The type of coverage information to be displayed.'
  )
  parser.add_argument(
      '-a',
      '--args',
      nargs='+',
      dest='test_args',
      required=False,
      help='The arguments to pass to the unit test executable being run.'
  )

  args = parser.parse_args()

  output = os.path.abspath(args.output)

  make_dirs(output)

  generate_all_reports = args.format == 'all'

  binaries, raw_profiles = collect_profiles(args)

  if len(raw_profiles) == 0:
    print('No raw profiles could be generated.')
    return -1

  binaries_flag = []
  for binary in binaries:
    binaries_flag.append('-object')
    binaries_flag.append(binary)

  llvm_bin_dir = get_llvm_bin_directory()

  merged_profile_path = merge_profiles(llvm_bin_dir, raw_profiles, output)

  if not os.path.exists(merged_profile_path):
    print('Could not generate or find merged profile %s.' % merged_profile_path)
    return -1

  llvm_cov_binary = os.path.join(llvm_bin_dir, 'llvm-cov')
  instr_profile_flag = '-instr-profile=%s' % merged_profile_path
  ignore_flags = '-ignore-filename-regex=third_party|unittest|fixture'

  # Generate the HTML report if specified.
  if generate_all_reports or args.format == 'html':
    print('Generating HTML report.')
    subprocess.check_call([llvm_cov_binary, 'show'] + binaries_flag + [
        instr_profile_flag,
        '-format=html',
        '-output-dir=%s' % output,
        '-tab-size=2',
        ignore_flags,
    ])
    print('Done.')

  # Generate a report summary if specified.
  if generate_all_reports or args.format == 'summary':
    print('Generating a summary report.')
    subprocess.check_call([llvm_cov_binary, 'report'] + binaries_flag + [
        instr_profile_flag,
        ignore_flags,
    ])
    print('Done.')

  # Generate a lcov summary if specified.
  if generate_all_reports or args.format == 'lcov':
    print('Generating LCOV report.')
    lcov_file = os.path.join(output, 'coverage.lcov')
    remove_if_exists(lcov_file)
    with open(lcov_file, 'w') as lcov_redirect:
      subprocess.check_call([llvm_cov_binary, 'export'] + binaries_flag + [
          instr_profile_flag,
          ignore_flags,
          '-format=lcov',
      ],
                            stdout=lcov_redirect)
    print('Done.')

  return 0


if __name__ == '__main__':
  sys.exit(main())
