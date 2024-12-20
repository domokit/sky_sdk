#!/usr/bin/env vpython3

# [VPYTHON:BEGIN]
# python_version: "3.8"
# wheel <
#   name: "infra/python/wheels/pyyaml/${platform}_${py_python}_${py_abi}"
#   version: "version:5.4.1.chromium.1"
# >
# [VPYTHON:END]

# Copyright (c) 2013, the Flutter project authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file.

import logging
import os
import sys

from typing import Any, Iterable, List, Mapping, Set

# The import is coming from vpython wheel and pylint cannot find it.
import yaml  # pylint: disable=import-error

# The imports are coming from fuchsia/test_scripts and pylint cannot find them
# without setting a global init-hook which is less favorable.
# But this file will be executed as part of the CI, its correctness of importing
# is guaranteed.

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), '../../tools/fuchsia/test_scripts/test/')
)

# pylint: disable=import-error, wrong-import-position
from bundled_test_runner import run_tests, TestCase
from common import DIR_SRC_ROOT
from compatible_utils import force_running_unattended

if len(sys.argv) == 2:
  VARIANT = sys.argv[1]
  sys.argv.pop()
elif len(sys.argv) == 1:
  VARIANT = 'fuchsia_debug_x64'
else:
  assert False, 'Expect only one parameter as the compile output directory.'
OUT_DIR = os.path.join(DIR_SRC_ROOT, 'out', VARIANT)


# Visible for testing
def resolve_packages(tests: Iterable[Mapping[str, Any]]) -> Set[str]:
  packages = set()
  for test in tests:
    if 'package' in test:
      packages.add(test['package'])
    else:
      assert 'packages' in test, \
             'Expect either one package or a list of packages'
      packages.update(test['packages'])
  resolved_packages = set()
  for package in packages:
    if package.endswith('-0.far'):
      # Make a symbolic link to match the name of the package itself without the
      # '-0.far' suffix.
      new_package = os.path.join(OUT_DIR, package.replace('-0.far', '.far'))
      try:
        # Remove the old one if it exists, usually happen on the devbox, so
        # ignore the FileNotFoundError.
        os.remove(new_package)
      except FileNotFoundError:
        pass
      os.symlink(package, new_package)
      resolved_packages.add(new_package)
    else:
      resolved_packages.add(os.path.join(OUT_DIR, package))
  return resolved_packages


# Visible for testing
def build_test_cases(tests: Iterable[Mapping[str, Any]]) -> List[TestCase]:
  test_cases = []
  for test in [t['test_command'] for t in tests]:
    assert test.startswith('test run ')
    test = test[len('test run '):]
    if ' -- ' in test:
      package, args = test.split(' -- ', 1)
      test_cases.append(TestCase(package=package, args=args))
    else:
      test_cases.append(TestCase(package=test))
  return test_cases


def main() -> int:
  logging.basicConfig(level=logging.INFO)
  logging.info('Running tests in %s', OUT_DIR)
  force_running_unattended()
  sys.argv.append('--out-dir=' + OUT_DIR)
  if VARIANT.endswith('_arm64') or VARIANT.endswith('_arm64_tester'):
    sys.argv.append('--product=terminal.qemu-arm64')

  sys.argv.append('--logs-dir=' + os.environ.get('FLUTTER_LOGS_DIR', '/tmp/log'))
  with open(os.path.join(os.path.dirname(__file__), 'test_suites.yaml'), 'r') as file:
    tests = yaml.safe_load(file)
  # TODO(zijiehe-google-com): Run all tests in release build,
  # https://github.com/flutter/flutter/issues/140179.
  def variant(test) -> bool:
    return 'variant' not in test or test['variant'] in VARIANT

  tests = [t for t in tests if variant(t)]
  for package in resolve_packages(tests):
    sys.argv.append('--packages=' + package)
  return run_tests(build_test_cases(tests))


if __name__ == '__main__':
  sys.exit(main())
