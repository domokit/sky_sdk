#!/usr/bin/env python3
#
# Copyright (c) 2015 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import os
import errno
import subprocess
import sys

PLUTIL = [
  '/usr/bin/env',
  'xcrun',
  'plutil'
]

IBTOOL = [
  '/usr/bin/env',
  'xcrun',
  'ibtool',
]


def MakeDirectories(path):
  try:
    os.makedirs(path)
  except OSError as exc:
    if exc.errno == errno.EEXIST and os.path.isdir(path):
      return 0
    else:
      return -1

  return 0


def ProcessInfoPlist(args):
  output_plist_file = os.path.abspath(os.path.join(args.output, 'Info.plist'))
  return subprocess.check_call( PLUTIL + [
    '-convert',
    'binary1',
    '-o',
    output_plist_file,
    '--',
    args.input,
  ])


def ProcessNIB(args):
  output_nib_file = os.path.join(os.path.abspath(args.output),
      "%s.nib" % os.path.splitext(os.path.basename(args.input))[0])

  return subprocess.check_call(IBTOOL + [
    '--module',
    args.module,
    '--auto-activate-custom-fonts',
    '--target-device',
    'mac',
    '--compile',
    output_nib_file,
    os.path.abspath(args.input),
  ])


def GenerateProjectStructure(args):
  application_path = os.path.join( args.dir, args.name + ".app", "Contents" )
  return MakeDirectories( application_path )


def Main():
  parser = argparse.ArgumentParser(description='A script that aids in '
                                   'the creation of an Mac application')

  subparsers = parser.add_subparsers()

  # Plist Parser

  plist_parser = subparsers.add_parser('plist',
                                       help='Process the Info.plist')
  plist_parser.set_defaults(func=ProcessInfoPlist)
  
  plist_parser.add_argument('-i', dest='input', help='The input plist path')
  plist_parser.add_argument('-o', dest='output', help='The output plist dir')

  # NIB Parser

  plist_parser = subparsers.add_parser('nib',
                                       help='Process a NIB file')
  plist_parser.set_defaults(func=ProcessNIB)
  
  plist_parser.add_argument('-i', dest='input', help='The input nib path')
  plist_parser.add_argument('-o', dest='output', help='The output nib dir')
  plist_parser.add_argument('-m', dest='module', help='The module name')

  # Directory Structure Parser

  dir_struct_parser = subparsers.add_parser('structure',
                      help='Creates the directory of an Mac application')

  dir_struct_parser.set_defaults(func=GenerateProjectStructure)

  dir_struct_parser.add_argument('-d', dest='dir', help='Out directory')
  dir_struct_parser.add_argument('-n', dest='name', help='App name')

  # Engage!

  args = parser.parse_args()

  return args.func(args)


if __name__ == '__main__':
  sys.exit(Main())
