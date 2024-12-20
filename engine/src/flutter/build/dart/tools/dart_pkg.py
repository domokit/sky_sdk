#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Utility for dart_pkg and dart_pkg_app rules"""

import argparse
import errno
import json
import os
import shutil
import subprocess
import sys

USE_LINKS = sys.platform != 'win32'

DART_ANALYZE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dart_analyze.py')


def dart_filter(path):
  if os.path.isdir(path):
    return True
  _, ext = os.path.splitext(path)
  # .dart includes '.mojom.dart'
  return ext == '.dart'


def ensure_dir_exists(path):
  abspath = os.path.abspath(path)
  if not os.path.exists(abspath):
    os.makedirs(abspath)


def has_pubspec_yaml(paths):
  for path in paths:
    _, filename = os.path.split(path)
    if filename == 'pubspec.yaml':
      return True
  return False


def link(from_root, to_root):
  ensure_dir_exists(os.path.dirname(to_root))
  try:
    os.unlink(to_root)
  except OSError as err:
    if err.errno == errno.ENOENT:
      pass

  try:
    os.symlink(from_root, to_root)
  except OSError as err:
    if err.errno == errno.EEXIST:
      pass


def copy(from_root, to_root, filter_func=None):
  if not os.path.exists(from_root):
    return
  if os.path.isfile(from_root):
    ensure_dir_exists(os.path.dirname(to_root))
    shutil.copy(from_root, to_root)
    return

  ensure_dir_exists(to_root)

  for root, dirs, files in os.walk(from_root):
    # filter_func expects paths not names, so wrap it to make them absolute.
    wrapped_filter = None
    if filter_func:
      wrapped_filter = lambda name, rt=root: filter_func(os.path.join(rt, name))

    for name in filter(wrapped_filter, files):
      from_path = os.path.join(root, name)
      root_rel_path = os.path.relpath(from_path, from_root)
      to_path = os.path.join(to_root, root_rel_path)
      to_dir = os.path.dirname(to_path)
      if not os.path.exists(to_dir):
        os.makedirs(to_dir)
      shutil.copy(from_path, to_path)

    dirs[:] = list(filter(wrapped_filter, dirs))


def copy_or_link(from_root, to_root, filter_func=None):
  if USE_LINKS:
    link(from_root, to_root)
  else:
    copy(from_root, to_root, filter_func)


def link_if_possible(from_root, to_root):
  if USE_LINKS:
    link(from_root, to_root)


def remove_if_exists(path):
  try:
    os.remove(path)
  except OSError as err:
    if err.errno != errno.ENOENT:
      raise


def list_files(from_root, filter_func=None):
  file_list = []
  for root, dirs, files in os.walk(from_root):
    # filter_func expects paths not names, so wrap it to make them absolute.
    wrapped_filter = None
    if filter_func:
      wrapped_filter = lambda name, rt=root: filter_func(os.path.join(rt, name))
    for name in filter(wrapped_filter, files):
      path = os.path.join(root, name)
      file_list.append(path)
    dirs[:] = list(filter(wrapped_filter, dirs))
  return file_list


def remove_broken_symlink(path):
  if not USE_LINKS:
    return
  try:
    link_path = os.readlink(path)
  except OSError as err:
    # Path was not a symlink.
    if err.errno == errno.EINVAL:
      pass
  else:
    if not os.path.exists(link_path):
      remove_if_exists(path)


def remove_broken_symlinks(root_dir):
  if not USE_LINKS:
    return
  for current_dir, _, child_files in os.walk(root_dir):
    for filename in child_files:
      path = os.path.join(current_dir, filename)
      remove_broken_symlink(path)


def analyze_entrypoints(dart_sdk, package_root, entrypoints):
  cmd = ['python', DART_ANALYZE]
  cmd.append('--dart-sdk')
  cmd.append(dart_sdk)
  cmd.append('--entrypoints')
  cmd.extend(entrypoints)
  cmd.append('--package-root')
  cmd.append(package_root)
  cmd.append('--no-hints')
  try:
    subprocess.check_output(cmd, stderr=subprocess.STDOUT)
  except subprocess.CalledProcessError as err:
    print('Failed analyzing %s' % entrypoints)
    print(err.output)
    return err.returncode
  return 0


def main():
  parser = argparse.ArgumentParser(description='Generate a dart-pkg')
  parser.add_argument(
      '--dart-sdk', action='store', metavar='dart_sdk', help='Path to the Dart SDK.'
  )
  parser.add_argument(
      '--package-name',
      action='store',
      metavar='package_name',
      help='Name of package',
      required=True
  )
  parser.add_argument(
      '--pkg-directory',
      metavar='pkg_directory',
      help='Directory where dart_pkg should go',
      required=True
  )
  parser.add_argument(
      '--package-root', metavar='package_root', help='packages/ directory', required=True
  )
  parser.add_argument('--stamp-file', metavar='stamp_file', help='timestamp file', required=True)
  parser.add_argument(
      '--entries-file', metavar='entries_file', help='script entries file', required=True
  )
  parser.add_argument(
      '--package-sources', metavar='package_sources', help='Package sources', nargs='+'
  )
  parser.add_argument(
      '--package-entrypoints',
      metavar='package_entrypoints',
      help='Package entry points for analyzer',
      nargs='*',
      default=[]
  )
  parser.add_argument(
      '--sdk-ext-directories',
      metavar='sdk_ext_directories',
      help='Directory containing .dart sources',
      nargs='*',
      default=[]
  )
  parser.add_argument(
      '--sdk-ext-files',
      metavar='sdk_ext_files',
      help='List of .dart files that are part of sdk_ext.',
      nargs='*',
      default=[]
  )
  parser.add_argument(
      '--sdk-ext-mappings',
      metavar='sdk_ext_mappings',
      help='Mappings for SDK extension libraries.',
      nargs='*',
      default=[]
  )
  parser.add_argument(
      '--read_only',
      action='store_true',
      dest='read_only',
      help='Package is a read only package.',
      default=False
  )
  args = parser.parse_args()

  # We must have a pubspec.yaml.
  assert has_pubspec_yaml(args.package_sources)

  target_dir = os.path.join(args.pkg_directory, args.package_name)
  target_packages_dir = os.path.join(target_dir, 'packages')
  lib_path = os.path.join(target_dir, 'lib')
  ensure_dir_exists(lib_path)

  mappings = {}
  for mapping in args.sdk_ext_mappings:
    library, path = mapping.split(',', 1)
    mappings[library] = '../sdk_ext/%s' % path

  sdkext_path = os.path.join(lib_path, '_sdkext')
  if mappings:
    with open(sdkext_path, 'w') as stream:
      json.dump(mappings, stream, sort_keys=True, indent=2, separators=(',', ': '))
  else:
    remove_if_exists(sdkext_path)

  # Copy or symlink package sources into pkg directory.
  common_source_prefix = os.path.dirname(os.path.commonprefix(args.package_sources))
  for source in args.package_sources:
    relative_source = os.path.relpath(source, common_source_prefix)
    target = os.path.join(target_dir, relative_source)
    copy_or_link(source, target)

  entrypoint_targets = []
  for source in args.package_entrypoints:
    relative_source = os.path.relpath(source, common_source_prefix)
    target = os.path.join(target_dir, relative_source)
    copy_or_link(source, target)
    entrypoint_targets.append(target)

  # Copy sdk-ext sources into pkg directory
  sdk_ext_dir = os.path.join(target_dir, 'sdk_ext')
  for directory in args.sdk_ext_directories:
    sdk_ext_sources = list_files(directory, dart_filter)
    common_prefix = os.path.commonprefix(sdk_ext_sources)
    for source in sdk_ext_sources:
      relative_source = os.path.relpath(source, common_prefix)
      target = os.path.join(sdk_ext_dir, relative_source)
      copy_or_link(source, target)

  common_source_prefix = os.path.dirname(os.path.commonprefix(args.sdk_ext_files))
  for source in args.sdk_ext_files:
    relative_source = os.path.relpath(source, common_source_prefix)
    target = os.path.join(sdk_ext_dir, relative_source)
    copy_or_link(source, target)

  # Symlink packages/
  package_path = os.path.join(args.package_root, args.package_name)
  copy_or_link(lib_path, package_path)

  # Link dart-pkg/$package/packages to dart-pkg/packages
  link_if_possible(args.package_root, target_packages_dir)

  # Remove any broken symlinks in target_dir and package root.
  remove_broken_symlinks(target_dir)
  remove_broken_symlinks(args.package_root)

  # If any entrypoints are defined, write them to disk so that the analyzer
  # test can find them.
  with open(args.entries_file, 'w') as file:
    for entrypoint in entrypoint_targets:
      file.write(entrypoint + '\n')

  # Write stamp file.
  with open(args.stamp_file, 'w'):
    pass

  return 0


if __name__ == '__main__':
  sys.exit(main())
