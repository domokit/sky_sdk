#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import json
import os
import stat
import sys
import zipfile


def _zip_dir(path, zip_file, prefix):
  path = path.rstrip('/\\')
  for root, directories, files in os.walk(path):
    for directory in directories:
      if os.path.islink(os.path.join(root, directory)):
        add_symlink(
            zip_file,
            os.path.join(root, directory),
            os.path.join(root.replace(path, prefix), directory),
        )
    for file in files:
      if os.path.islink(os.path.join(root, file)):
        add_symlink(
            zip_file, os.path.join(root, file), os.path.join(root.replace(path, prefix), file)
        )
        continue
      zip_file.write(os.path.join(root, file), os.path.join(root.replace(path, prefix), file))


def add_symlink(zip_file, source, target):
  """Adds a symlink to a zip file.

  Args:
    zip_file: The ZipFile obj where the symlink will be added.
    source: The full path to the symlink.
    target: The target path for the symlink within the zip file.
  """
  zip_info = zipfile.ZipInfo(target)
  zip_info.create_system = 3  # Unix like system
  unix_st_mode = (
      stat.S_IFLNK | stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | stat.S_IRGRP
      | stat.S_IWGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IWOTH | stat.S_IXOTH
  )
  zip_info.external_attr = unix_st_mode << 16
  zip_file.writestr(zip_info, os.readlink(source))


def main(args):
  zip_file = zipfile.ZipFile(args.output, 'w', zipfile.ZIP_DEFLATED)
  if args.source_file:
    with open(args.source_file) as source_file:
      file_dict_list = json.load(source_file)
      for file_dict in file_dict_list:
        if os.path.islink(file_dict['source']):
          add_symlink(zip_file, file_dict['source'], file_dict['destination'])
          continue
        if os.path.isdir(file_dict['source']):
          _zip_dir(file_dict['source'], zip_file, file_dict['destination'])
        else:
          zip_file.write(file_dict['source'], file_dict['destination'])
  else:
    for path, archive_name in args.input_pairs:
      if os.path.islink(path):
        add_symlink(zip_file, path, archive_name)
        continue
      if os.path.isdir(path):
        _zip_dir(path, zip_file, archive_name)
      else:
        zip_file.write(path, archive_name)
  zip_file.close()


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='This script creates zip files.')
  parser.add_argument('-o', dest='output', action='store', help='The name of the output zip file.')
  parser.add_argument(
      '-i',
      dest='input_pairs',
      nargs=2,
      action='append',
      help='The input file and its destination location in the zip archive.'
  )
  parser.add_argument(
      '-f', dest='source_file', action='store', help='The path to the file list to zip.'
  )
  sys.exit(main(parser.parse_args()))
