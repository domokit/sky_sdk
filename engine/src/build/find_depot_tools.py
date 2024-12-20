#!/usr/bin/env python3
# Copyright 2011 The Chromium Authors
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
"""Small utility function to find depot_tools and add it to the python path.

Will throw an ImportError exception if depot_tools can't be found since it
imports breakpad.

This can also be used as a standalone script to print out the depot_tools
directory location.
"""


import os
import sys


# Path to //src
SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))


def IsRealDepotTools(path):
  expanded_path = os.path.expanduser(path)
  return os.path.isfile(os.path.join(expanded_path, 'gclient.py'))


def add_depot_tools_to_path():
  """Search for depot_tools and add it to sys.path."""
  # First, check if we have a DEPS'd in "depot_tools".
  deps_depot_tools = os.path.join(SRC, 'flutter', 'third_party', 'depot_tools')
  if IsRealDepotTools(deps_depot_tools):
    # Put the pinned version at the start of the sys.path, in case there
    # are other non-pinned versions already on the sys.path.
    sys.path.insert(0, deps_depot_tools)
    return deps_depot_tools

  # Then look if depot_tools is already in PYTHONPATH.
  for i in sys.path:
    if i.rstrip(os.sep).endswith('depot_tools') and IsRealDepotTools(i):
      return i
  # Then look if depot_tools is in PATH, common case.
  for i in os.environ['PATH'].split(os.pathsep):
    if IsRealDepotTools(i):
      sys.path.append(i.rstrip(os.sep))
      return i
  # Rare case, it's not even in PATH, look upward up to root.
  root_dir = os.path.dirname(os.path.abspath(__file__))
  previous_dir = os.path.abspath(__file__)
  while root_dir and root_dir != previous_dir:
    i = os.path.join(root_dir, 'depot_tools')
    if IsRealDepotTools(i):
      sys.path.append(i)
      return i
    previous_dir = root_dir
    root_dir = os.path.dirname(root_dir)
  print('Failed to find depot_tools', file=sys.stderr)
  return None

DEPOT_TOOLS_PATH = add_depot_tools_to_path()

# pylint: disable=W0611
import breakpad


def main():
  if DEPOT_TOOLS_PATH is None:
    return 1
  print(DEPOT_TOOLS_PATH)
  return 0


if __name__ == '__main__':
  sys.exit(main())
