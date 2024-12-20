#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import os
import subprocess
import sys

ANDROID_LOG_CLASS = 'android.util.Log'
FLUTTER_LOG_CLASS = 'io.flutter.Log'

ANDROIDX_TRACE_CLASS = 'androidx.tracing.Trace'
ANDROID_TRACE_CLASS = 'android.tracing.Trace'
FLUTTER_TRACE_CLASS = 'io.flutter.util.TraceSection'

ANDROID_BUILD_VERSION_CODE_CLASS = 'VERSION_CODES'


def CheckBadFiles(bad_files, bad_class, good_class):
  if bad_files:
    print('')
    print('Illegal import %s detected in the following files:' % bad_class)
    for bad_file in bad_files:
      print('  - ' + bad_file)
    print('Use %s instead.' % good_class)
    print('')
    return True

  return False


def main():
  parser = argparse.ArgumentParser(
      description='Checks Flutter Android library for forbidden imports'
  )
  parser.add_argument('--stamp', type=str, required=True)
  parser.add_argument('--files', type=str, required=True, nargs='+')
  args = parser.parse_args()

  open(args.stamp, 'a').close()

  bad_log_files = []
  bad_trace_files = []
  bad_version_codes_files = []

  for file in args.files:
    if (file.endswith(os.path.join('io', 'flutter', 'Log.java')) or
        file.endswith(os.path.join('io', 'flutter', 'util', 'TraceSection.java')) or
        file.endswith(os.path.join('io', 'flutter', 'Build.java'))):
      continue
    with open(file) as f:
      contents = f.read()
      if ANDROID_LOG_CLASS in contents:
        bad_log_files.append(file)
      if ANDROIDX_TRACE_CLASS in contents or ANDROID_TRACE_CLASS in contents:
        bad_trace_files.append(file)
      if ANDROID_BUILD_VERSION_CODE_CLASS in contents:
        bad_version_codes_files.append(file)

  # Flutter's Log class allows additional configuration around verbosity.

  # Flutter's tracing class makes sure we do not violate string lengths that
  # cause crashes at runtime.

  # Flutter's Build.API_LEVELS class is clearer to read about which API version
  # is used.
  has_bad_files = CheckBadFiles(bad_log_files, ANDROID_LOG_CLASS,
                                FLUTTER_LOG_CLASS) or CheckBadFiles(
                                    bad_trace_files, 'android[x].tracing.Trace', FLUTTER_TRACE_CLASS
                                ) or CheckBadFiles(
                                    bad_version_codes_files, 'android.os.Build.VERSION_CODES',
                                    'io.flutter.Build.API_LEVELS'
                                )

  if has_bad_files:
    return 1

  return 0


if __name__ == '__main__':
  sys.exit(main())
