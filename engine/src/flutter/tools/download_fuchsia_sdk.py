#!/usr/bin/env python3
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# The return code of this script will always be 0, even if there is an error,
# unless the --fail-loudly flag is passed.

import argparse
import tarfile
import json
import os
import shutil
import subprocess
import sys

SRC_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FUCHSIA_SDK_DIR = os.path.join(SRC_ROOT, 'fuchsia', 'sdk')
FLUTTER_DIR = os.path.join(SRC_ROOT, 'flutter')


# Prints to stderr.
def eprint(*args, **kwargs):
  print(*args, file=sys.stderr, **kwargs)


def FileNameForSdkPath(sdk_path):
  return sdk_path.split('/')[-1]


def DownloadFuchsiaSDKFromGCS(sdk_path, verbose):
  file = FileNameForSdkPath(sdk_path)
  url = 'https://storage.googleapis.com/fuchsia-artifacts/{}'.format(sdk_path)
  dest = os.path.join(FUCHSIA_SDK_DIR, file)

  if verbose:
    print('Fuchsia SDK url: "%s"' % url)
    print('Fuchsia SDK destination path: "%s"' % dest)

  if os.path.isfile(dest):
    os.unlink(dest)

  # Ensure destination folder exists.
  os.makedirs(FUCHSIA_SDK_DIR, exist_ok=True)
  curl_command = [
      'curl',
      '--retry',
      '3',
      '--continue-at',
      '-',
      '--location',
      '--output',
      dest,
      url,
  ]
  if verbose:
    print('Running: "%s"' % (' '.join(curl_command)))
  curl_result = subprocess.run(
      curl_command,
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE,
      universal_newlines=True,
  )
  if curl_result.returncode == 0 and verbose:
    print('curl output:stdout:\n{}\nstderr:\n{}'.format(
        curl_result.stdout,
        curl_result.stderr,
    ))
  elif curl_result.returncode != 0:
    eprint(
        'Failed to download: stdout:\n{}\nstderr:\n{}'.format(
            curl_result.stdout,
            curl_result.stderr,
        )
    )
    return None

  return dest


def OnErrorRmTree(func, path, exc_info):
  """
  Error handler for ``shutil.rmtree``.

  If the error is due to an access error (read only file)
  it attempts to add write permission and then retries.
  If the error is for another reason it re-raises the error.

  Usage : ``shutil.rmtree(path, onerror=onerror)``
  """
  import stat
  # Is the error an access error?
  if not os.access(path, os.W_OK):
    os.chmod(path, stat.S_IWUSR)
    func(path)
  else:
    raise


def ExtractGzipArchive(archive, host_os, verbose):
  sdk_dest = os.path.join(FUCHSIA_SDK_DIR, host_os)
  if os.path.isdir(sdk_dest):
    shutil.rmtree(sdk_dest, onerror=OnErrorRmTree)

  extract_dest = os.path.join(FUCHSIA_SDK_DIR, 'temp')
  if os.path.isdir(extract_dest):
    shutil.rmtree(extract_dest, onerror=OnErrorRmTree)
  os.makedirs(extract_dest, exist_ok=True)

  if verbose:
    print('Extracting "%s" to "%s"' % (archive, extract_dest))

  with tarfile.open(archive, 'r') as z:
    z.extractall(extract_dest)

  shutil.move(extract_dest, sdk_dest)


def Main():
  parser = argparse.ArgumentParser()
  parser.add_argument(
      '--fail-loudly',
      action='store_true',
      default=False,
      help="Return an error code if a prebuilt couldn't be fetched and extracted"
  )

  parser.add_argument(
      '--verbose',
      action='store_true',
      default='LUCI_CONTEXT' in os.environ,
      help='Emit verbose output'
  )

  parser.add_argument('--host-os', help='The host os')

  parser.add_argument('--fuchsia-sdk-path', help='The path in gcs to the fuchsia sdk to download')

  args = parser.parse_args()
  fail_loudly = 1 if args.fail_loudly else 0
  verbose = args.verbose
  host_os = args.host_os
  fuchsia_sdk_path = args.fuchsia_sdk_path

  if fuchsia_sdk_path is None:
    eprint('sdk_path can not be empty')
    return fail_loudly

  archive = DownloadFuchsiaSDKFromGCS(fuchsia_sdk_path, verbose)
  if archive is None:
    eprint('Failed to download SDK from %s' % fuchsia_sdk_path)
    return fail_loudly

  ExtractGzipArchive(archive, host_os, verbose)

  success = True
  return 0 if success else fail_loudly


if __name__ == '__main__':
  sys.exit(Main())
