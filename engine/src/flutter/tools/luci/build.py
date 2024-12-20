#!/usr/bin/env python3
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# A script for re-running prod builds on LUCI
#
# Usage:
# $ build.py --commit [Engine commit hash] --builder [builder name]
#
# NOTE: This script requires python3.7 or later.
#

import argparse
import os
import re
import subprocess
import sys


def GetAllBuilders():
  curl_command = [
      'curl',
      'https://ci.chromium.org/p/flutter/g/engine/builders',
  ]
  curl_result = subprocess.run(
      curl_command,
      universal_newlines=True,
      capture_output=True,
  )
  if curl_result.returncode != 0:
    print('Failed to fetch builder list: stderr:\n%s' % curl_result.stderr)
    return []
  sed_command = [
      'sed',
      '-En',
      's:.*aria-label="builder buildbucket/luci\\.flutter\\.prod/([^/]+)".*:\\1:p',
  ]
  sed_result = subprocess.run(
      sed_command,
      input=curl_result.stdout,
      capture_output=True,
      universal_newlines=True,
  )
  if sed_result.returncode != 0:
    print('Failed to fetch builder list: stderr:\n%s' % sed_result.stderr)
  return list(set(sed_result.stdout.splitlines()))


def Main():
  parser = argparse.ArgumentParser(description='Reruns Engine LUCI prod builds')
  parser.add_argument(
      '--force-upload',
      action='store_true',
      default=False,
      help='Force artifact upload, overwriting existing artifacts.'
  )
  parser.add_argument('--all', action='store_true', default=False, help='Re-run all builds.')
  parser.add_argument('--builder', type=str, help='The builer to rerun.')
  parser.add_argument('--commit', type=str, required=True, help='The commit to rerun.')
  parser.add_argument(
      '--dry-run', action='store_true', help='Print what would be done, but do nothing.'
  )
  args = parser.parse_args()

  if 'help' in vars(args) and args.help:
    parser.print_help()
    return 0

  if args.all:
    builders = GetAllBuilders()
  elif args.builder == None:
    print('Either --builder or --all is required.')
    return 1
  else:
    builders = [args.builder]

  auth_command = [
      'gcloud',
      'auth',
      'print-identity-token',
  ]
  auth_result = subprocess.run(
      auth_command,
      universal_newlines=True,
      capture_output=True,
  )
  if auth_result.returncode != 0:
    print('Auth failed:\nstdout:\n%s\nstderr:\n%s' % (auth_result.stdout, auth_result.stderr))
    return 1
  auth_token = auth_result.stdout.rstrip()

  for builder in builders:
    if args.force_upload:
      params = (
          '{"Commit": "%s", "Builder": "%s", "Repo": "engine", "Properties": {"force_upload":true}}'
          % (args.commit, builder)
      )
    else:
      params = '{"Commit": "%s", "Builder": "%s", "Repo": "engine"}' % (args.commit, builder)
    curl_command = [
        'curl',
        'http://flutter-dashboard.appspot.com/api/reset-prod-task',
        "-d %s" % params,
        '-H',
        'X-Flutter-IdToken: %s' % auth_token,
    ]
    if args.dry_run:
      print('Running: %s' % ' '.join(curl_command))
    else:
      result = subprocess.run(curl_command)
      if result.returncode != 0:
        print('Trigger for %s failed. Aborting.' % builder)
        return 1

  return 0


if __name__ == '__main__':
  sys.exit(Main())
