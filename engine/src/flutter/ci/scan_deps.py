#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#
# Usage: scan_deps.py --deps <DEPS file> --output <parsed lockfile>
#
# This script extracts the dependencies provided from the DEPS file and
# finds the appropriate git commit hash per dependency for osv-scanner
# to use in checking for vulnerabilities.
# It is expected that the lockfile output of this script is then
# uploaded using GitHub actions to be used by the osv-scanner reusable action.

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from compatibility_helper import byte_str_decode

SCRIPT_DIR = os.path.dirname(sys.argv[0])
CHECKOUT_ROOT = os.path.realpath(os.path.join(SCRIPT_DIR, '..'))
CHROMIUM_README_FILE = 'third_party/accessibility/README.md'
CHROMIUM_README_COMMIT_LINE = 4  # The fifth line will always contain the commit hash.
CHROMIUM = 'https://chromium.googlesource.com/chromium/src'
DEP_CLONE_DIR = CHECKOUT_ROOT + '/clone-test'
DEPS = os.path.join(CHECKOUT_ROOT, 'DEPS')
UPSTREAM_PREFIX = 'upstream_'


# Used in parsing the DEPS file.
class VarImpl:
  _env_vars = {
      'host_cpu': 'x64',
      'host_os': 'linux',
  }

  def __init__(self, local_scope):
    self._local_scope = local_scope

  def lookup(self, var_name):
    """Implements the Var syntax."""
    if var_name in self._local_scope.get('vars', {}):
      return self._local_scope['vars'][var_name]
    # Inject default values for env variables.
    if var_name in self._env_vars:
      return self._env_vars[var_name]
    raise Exception('Var is not defined: %s' % var_name)


def extract_deps(deps_file):
  local_scope = {}
  var = VarImpl(local_scope)
  global_scope = {
      'Var': var.lookup,
      'deps_os': {},
  }
  # Read the content.
  with open(deps_file, 'r') as file:
    deps_content = file.read()

  # Eval the content.
  exec(deps_content, global_scope, local_scope)

  if not os.path.exists(DEP_CLONE_DIR):
    os.mkdir(DEP_CLONE_DIR)  # Clone deps with upstream into temporary dir.

  # Extract the deps and filter.
  deps = local_scope.get('deps', {})
  deps_list = local_scope.get('vars')
  filtered_osv_deps = []
  for _, dep in deps.items():
    # We currently do not support packages or cipd which are represented
    # as dictionaries.
    if not isinstance(dep, str):
      continue

    dep_split = dep.rsplit('@', 1)
    ancestor_result = get_common_ancestor([dep_split[0], dep_split[1]], deps_list)
    if ancestor_result:
      filtered_osv_deps.append({
          'package': {'name': ancestor_result[1], 'commit': ancestor_result[0]}
      })

  try:
    # Clean up cloned upstream dependency directory.
    shutil.rmtree(DEP_CLONE_DIR)  # Use shutil.rmtree since dir could be non-empty.
  except OSError as clone_dir_error:
    print('Error cleaning up clone directory: %s : %s' % (DEP_CLONE_DIR, clone_dir_error.strerror))

  osv_result = {
      'packageSource': {'path': deps_file, 'type': 'lockfile'}, 'packages': filtered_osv_deps
  }
  return osv_result


def parse_readme():
  """
  Opens the Flutter Accessibility Library README and uses the commit hash
  found in the README to check for viulnerabilities.
  The commit hash in this README will always be in the same format
  """
  file_path = os.path.join(CHECKOUT_ROOT, CHROMIUM_README_FILE)
  with open(file_path) as file:
    # Read the content of the file opened.
    content = file.readlines()
    commit_line = content[CHROMIUM_README_COMMIT_LINE]
    commit = re.search(r'(?<=\[).*(?=\])', commit_line)

    osv_result = {
        'packageSource': {'path': file_path, 'type': 'lockfile'},
        'packages': [{'package': {'name': CHROMIUM, 'commit': commit.group()}}]
    }

    return osv_result


def get_common_ancestor(dep, deps_list):
  """
  Given an input of a mirrored dep,
  compare to the mapping of deps to their upstream
  in DEPS and find a common ancestor
  commit SHA value.

  This is done by first cloning the mirrored dep,
  then a branch which tracks the upstream.
  From there,  git merge-base operates using the HEAD
  commit SHA of the upstream branch and the pinned
  SHA value of the mirrored branch
  """
  # dep[0] contains the mirror repo.
  # dep[1] contains the mirror's pinned SHA.
  # upstream is the origin repo.
  dep_name = dep[0].split('/')[-1].split('.')[0]
  if UPSTREAM_PREFIX + dep_name not in deps_list:
    print('did not find dep: ' + dep_name)
    return None
  try:
    # Get the upstream URL from the mapping in DEPS file.
    upstream = deps_list.get(UPSTREAM_PREFIX + dep_name)
    temp_dep_dir = DEP_CLONE_DIR + '/' + dep_name
    # Clone dependency from mirror.
    subprocess.check_output(['git', 'clone', '--quiet', '--', dep[0], dep_name], cwd=DEP_CLONE_DIR)

    # Create branch that will track the upstream dep.
    print('attempting to add upstream remote from: {upstream}'.format(upstream=upstream))
    subprocess.check_output(['git', 'remote', 'add', 'upstream', upstream], cwd=temp_dep_dir)
    subprocess.check_output(['git', 'fetch', '--quiet', 'upstream'], cwd=temp_dep_dir)
    # Get name of the default branch for upstream (e.g. main/master/etc.).
    default_branch = subprocess.check_output(
        'git remote show upstream ' + "| sed -n \'/HEAD branch/s/.*: //p\'",
        cwd=temp_dep_dir,
        shell=True
    )
    default_branch = byte_str_decode(default_branch)
    default_branch = default_branch.strip()

    # Make upstream branch track the upstream dep.
    subprocess.check_output([
        'git', 'checkout', '--force', '-b', 'upstream', '--track', 'upstream/' + default_branch
    ],
                            cwd=temp_dep_dir)
    # Get the most recent commit from default branch of upstream.
    commit = subprocess.check_output(
        'git for-each-ref ' + "--format=\'%(objectname:short)\' refs/heads/upstream",
        cwd=temp_dep_dir,
        shell=True
    )
    commit = byte_str_decode(commit)
    commit = commit.strip()

    # Perform merge-base on most recent default branch commit and pinned mirror commit.
    ancestor_commit = subprocess.check_output(
        'git merge-base {commit} {depUrl}'.format(commit=commit, depUrl=dep[1]),
        cwd=temp_dep_dir,
        shell=True
    )
    ancestor_commit = byte_str_decode(ancestor_commit)
    ancestor_commit = ancestor_commit.strip()
    print('Ancestor commit: ' + ancestor_commit)
    return ancestor_commit, upstream
  except subprocess.CalledProcessError as error:
    print(
        "Subprocess command '{0}' failed with exit code: {1}.".format(
            error.cmd, str(error.returncode)
        )
    )
    if error.output:
      print("Subprocess error output: '{0}'".format(error.output))
  return None


def parse_args(args):
  args = args[1:]
  parser = argparse.ArgumentParser(description='A script to find common ancestor commit SHAs')

  parser.add_argument(
      '--deps',
      '-d',
      type=str,
      help='Input DEPS file to extract.',
      default=os.path.join(CHECKOUT_ROOT, 'DEPS')
  )
  parser.add_argument(
      '--output',
      '-o',
      type=str,
      help='Output osv-scanner compatible deps file.',
      default=os.path.join(CHECKOUT_ROOT, 'osv-lockfile.json')
  )

  return parser.parse_args(args)


def write_manifest(deps, manifest_file):
  output = {'results': deps}
  print(json.dumps(output, indent=2))
  with open(manifest_file, 'w') as manifest:
    json.dump(output, manifest, indent=2)


def main(argv):
  args = parse_args(argv)
  deps = extract_deps(args.deps)
  readme_deps = parse_readme()
  write_manifest([deps, readme_deps], args.output)
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv))
