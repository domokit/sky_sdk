#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import glob
import re
import os
import subprocess
import sys
from compatibility_helper import byte_str_decode

if 'STORAGE_BUCKET' not in os.environ:
  print('The GCP storage bucket must be provided as an environment variable.')
  sys.exit(1)
BUCKET = os.environ['STORAGE_BUCKET']

if 'GCP_PROJECT' not in os.environ:
  print('The GCP project must be provided as an environment variable.')
  sys.exit(1)
PROJECT = os.environ['GCP_PROJECT']

# Exit codes returned by the FTL command that signal an infrastructure failure.
FTL_INFRA_FAILURE_CODES = [1, 15, 20]

# Maximum number of retries done if an infrastructure failure occurs.
MAX_RETRY_ATTEMPTS = 2

script_dir = os.path.dirname(os.path.realpath(__file__))
buildroot_dir = os.path.abspath(os.path.join(script_dir, '..', '..'))
out_dir = os.path.join(buildroot_dir, 'out')
error_re = re.compile(r'[EF]/flutter.+')


def run_firebase_test(apk, results_dir):
  # game-loop tests are meant for OpenGL apps.
  # This type of test will give the application a handle to a file, and
  # we'll write the timeline JSON to that file.
  # See https://firebase.google.com/docs/test-lab/android/game-loop
  # Pixel 5. As of this commit, this is a highly available device in FTL.
  process = subprocess.Popen(
      [
          'gcloud',
          '--project',
          PROJECT,
          'firebase',
          'test',
          'android',
          'run',
          '--type',
          'game-loop',
          '--app',
          apk,
          '--timeout',
          '2m',
          '--results-bucket',
          BUCKET,
          '--results-dir',
          results_dir,
          '--device',
          'model=shiba,version=34',
      ],
      stdout=subprocess.PIPE,
      stderr=subprocess.STDOUT,
      universal_newlines=True,
  )
  return process


def check_logcat(results_dir):
  logcat = subprocess.check_output(['gsutil', 'cat', '%s/%s/*/logcat' % (BUCKET, results_dir)])
  logcat = byte_str_decode(logcat)
  if not logcat:
    sys.exit(1)

  logcat_matches = error_re.findall(logcat)
  if logcat_matches:
    print('Errors in logcat:')
    print(logcat_matches)
    sys.exit(1)


def check_timeline(results_dir):
  gsutil_du = subprocess.check_output([
      'gsutil', 'du',
      '%s/%s/*/game_loop_results/results_scenario_0.json' % (BUCKET, results_dir)
  ])
  gsutil_du = byte_str_decode(gsutil_du)
  gsutil_du = gsutil_du.strip()
  if gsutil_du == '0':
    print('Failed to produce a timeline.')
    sys.exit(1)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument(
      '--variant',
      dest='variant',
      action='store',
      default='android_profile_arm64',
      help='The engine variant to run tests for.'
  )
  parser.add_argument(
      '--build-id',
      default=os.environ.get('SWARMING_TASK_ID', 'local_test'),
      help='A unique build identifier for this test. Used to sort results in the GCS bucket.'
  )

  args = parser.parse_args()

  apks_dir = os.path.join(out_dir, args.variant, 'firebase_apks')
  apks = set(glob.glob('%s/*.apk' % apks_dir))

  if not apks:
    print('No APKs found at %s' % apks_dir)
    return 1

  git_revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=script_dir)
  git_revision = byte_str_decode(git_revision)
  git_revision = git_revision.strip()

  for retry in range(MAX_RETRY_ATTEMPTS):
    if retry > 0:
      print('Retrying %s' % apks)

    results = []
    for apk in sorted(apks):
      results_dir = '%s/%s/%s' % (os.path.basename(apk), git_revision, args.build_id)
      process = run_firebase_test(apk, results_dir)
      results.append((apk, results_dir, process))

    for apk, results_dir, process in results:
      print('===== Test output for %s' % apk)
      for line in iter(process.stdout.readline, ''):
        print(line.strip())

      return_code = process.wait()
      if return_code in FTL_INFRA_FAILURE_CODES:
        print('Firebase test %s failed with infrastructure error code: %s' % (apk, return_code))
        continue
      if return_code != 0:
        print('Firebase test %s failed with code: %s' % (apk, return_code))
        sys.exit(return_code)

      print('Checking logcat for %s' % results_dir)
      check_logcat(results_dir)
      # scenario_app produces a timeline, but the android image test does not.
      if 'scenario' in apk:
        print('Checking timeline for %s' % results_dir)
        check_timeline(results_dir)

      apks.remove(apk)

    if not apks:
      break

  return 0


if __name__ == '__main__':
  sys.exit(main())
