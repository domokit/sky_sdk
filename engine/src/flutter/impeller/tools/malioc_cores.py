#!/usr/bin/env vpython3
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import json
import os
import subprocess
import sys

# This script parses the JSON output produced by malioc about GPU cores,
# and outputs it in a form that can be consumed by GN


def parse_args(argv):
  parser = argparse.ArgumentParser(
      description='This script sanitizes GPU core info output from malioc',
  )
  parser.add_argument(
      '--malioc',
      '-m',
      type=str,
      help='The path to malioc.',
  )
  parser.add_argument(
      '--output',
      '-o',
      type=str,
      help='The output path.',
  )
  return parser.parse_args(argv)


def validate_args(args):
  if not args.malioc or not os.path.isfile(args.malioc):
    print('The --malioc argument must refer to the malioc binary.')
    return False
  return True


def malioc_core_list(malioc):
  malioc_cores = subprocess.check_output(
      [malioc, '--list', '--format', 'json'],
      stderr=subprocess.STDOUT,
  )
  cores_json = json.loads(malioc_cores)
  cores = []
  for core in cores_json['cores']:
    cores.append(core['core'])
  return cores


def malioc_core_info(malioc, core):
  malioc_info = subprocess.check_output(
      [malioc, '--info', '--core', core, '--format', 'json'],
      stderr=subprocess.STDOUT,
  )
  info_json = json.loads(malioc_info)

  apis = info_json['apis']

  opengles_max_version = 0
  if 'opengles' in apis:
    opengles = apis['opengles']
    if opengles['max_version'] is not None:
      opengles_max_version = opengles['max_version']
  vulkan_max_version = 0
  if 'vulkan' in apis:
    vulkan = apis['vulkan']
    if vulkan['max_version'] is not None:
      vulkan_max_version = int(vulkan['max_version'] * 100)

  info = {
      'core': core,
      'opengles_max_version': opengles_max_version,
      'vulkan_max_version': vulkan_max_version,
  }
  return info


def main(argv):
  args = parse_args(argv[1:])
  if not validate_args(args):
    return 1

  infos = []
  for core in malioc_core_list(args.malioc):
    infos.append(malioc_core_info(args.malioc, core))

  if args.output:
    with open(args.output, 'w') as file:
      json.dump(infos, file, sort_keys=True)
  else:
    print(infos)

  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv))
