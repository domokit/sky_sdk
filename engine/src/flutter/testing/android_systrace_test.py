#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import os
import subprocess
import sys

BUILDROOT_DIR = os.path.abspath(os.path.join(os.path.realpath(__file__), '..', '..', '..'))

PERFETTO_SESSION_KEY = 'session1'
PERFETTO_TRACE_FILE = '/data/misc/perfetto-traces/trace'
PERFETTO_CONFIG = """
write_into_file: true
file_write_period_ms: 1000000000
flush_period_ms: 1000

buffers: {
    size_kb: 129024
}
data_sources: {
    config {
        name: "linux.ftrace"
        ftrace_config {
            ftrace_events: "ftrace/print"
            atrace_apps: "%s"
        }
    }
}
"""


def install_apk(apk_path, package_name, adb_path='adb'):
  print('Installing APK')
  subprocess.check_output([adb_path, 'shell', 'am', 'force-stop', package_name])
  # Allowed to fail if APK was never installed.
  subprocess.call([adb_path, 'uninstall', package_name], stdout=subprocess.DEVNULL)
  subprocess.check_output([adb_path, 'install', apk_path])


def start_perfetto(package_name, adb_path='adb'):
  print('Starting trace')
  cmd = [
      adb_path, 'shell', 'echo', "'" + PERFETTO_CONFIG % package_name + "'", '|', 'perfetto', '-c',
      '-', '--txt', '-o', PERFETTO_TRACE_FILE, '--detach', PERFETTO_SESSION_KEY
  ]

  subprocess.check_output(cmd, stderr=subprocess.STDOUT)


def launch_package(package_name, activity_name, adb_path='adb'):
  print('Scanning logcat')
  subprocess.check_output([adb_path, 'logcat', '-c'], stderr=subprocess.STDOUT)
  logcat = subprocess.Popen([adb_path, 'logcat'],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            universal_newlines=True)

  print('Launching %s (%s)' % (package_name, activity_name))
  subprocess.check_output([
      adb_path, 'shell', 'am ', 'start', '-n',
      '%s/%s' % (package_name, activity_name)
  ],
                          stderr=subprocess.STDOUT)
  for line in logcat.stdout:
    print('>>>>>>>> ' + line.strip())
    if ('Observatory listening' in line) or ('Dart VM service is listening' in line):
      logcat.kill()
      break


def collect_and_validate_trace(adb_path='adb'):
  print('Fetching trace')
  subprocess.check_output([
      adb_path, 'shell', 'perfetto', '--attach', PERFETTO_SESSION_KEY, '--stop'
  ],
                          stderr=subprocess.STDOUT)
  subprocess.check_output([adb_path, 'pull', PERFETTO_TRACE_FILE, 'trace.pb'],
                          stderr=subprocess.STDOUT)

  print('Validating trace')
  traceconv = os.path.join(
      BUILDROOT_DIR, 'flutter', 'third_party', 'android_tools', 'trace_to_text', 'trace_to_text'
  )
  traceconv_output = subprocess.check_output([traceconv, 'systrace', 'trace.pb'],
                                             stderr=subprocess.STDOUT,
                                             universal_newlines=True)

  print('Trace output:')
  print(traceconv_output)

  if 'ShellSetupUISubsystem' in traceconv_output:
    return 0

  print('Trace did not contain ShellSetupUISubsystem, failing.')
  return 1


def main():
  parser = argparse.ArgumentParser()

  parser.add_argument(
      '--apk-path', dest='apk_path', action='store', help='Provide the path to the APK to install'
  )
  parser.add_argument(
      '--package-name',
      dest='package_name',
      action='store',
      help='The package name of the APK, e.g. dev.flutter.scenarios'
  )
  parser.add_argument(
      '--activity-name',
      dest='activity_name',
      action='store',
      help='The activity to launch as it appears in AndroidManifest.xml, '
      'e.g. .PlatformViewsActivity'
  )
  parser.add_argument(
      '--adb-path',
      dest='adb_path',
      action='store',
      default='adb',
      help='Provide the path of adb used for android tests. '
      'By default it looks on $PATH.'
  )

  args = parser.parse_args()

  android_api_level = subprocess.check_output([
      args.adb_path, 'shell', 'getprop', 'ro.build.version.sdk'
  ],
                                              text=True).strip()
  if int(android_api_level) < 29:
    print('Android API %s detected. This script requires API 29 or above.' % android_api_level)
    return 0

  install_apk(args.apk_path, args.package_name, args.adb_path)
  start_perfetto(args.package_name, args.adb_path)
  launch_package(args.package_name, args.activity_name, args.adb_path)
  return collect_and_validate_trace(args.adb_path)


if __name__ == '__main__':
  sys.exit(main())
