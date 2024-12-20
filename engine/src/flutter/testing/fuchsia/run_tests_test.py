#!/usr/bin/env vpython3

# [VPYTHON:BEGIN]
# python_version: "3.8"
# wheel <
#   name: "infra/python/wheels/pyyaml/${platform}_${py_python}_${py_abi}"
#   version: "version:5.4.1.chromium.1"
# >
# [VPYTHON:END]

# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import os
import shutil
import unittest

from pathlib import Path

# pylint is looking for a wrong //testing/run_tests.py.
# pylint: disable=no-member
import run_tests

run_tests.OUT_DIR = '/tmp/out/fuchsia_debug_x64'
os.makedirs(run_tests.OUT_DIR, exist_ok=True)


class RunTestsTest(unittest.TestCase):

  def test_resolve_both_package_and_packages(self):
    packages = run_tests.resolve_packages([{'package': 'abc'}, {'packages': ['abc', 'def']}])
    self.assertEqual(
        packages, {os.path.join(run_tests.OUT_DIR, 'abc'),
                   os.path.join(run_tests.OUT_DIR, 'def')}
    )

  def test_resolve_package_make_symbolic_link(self):
    Path(os.path.join(run_tests.OUT_DIR, 'abc-0.far')).touch()
    packages = run_tests.resolve_packages([
        {'package': 'abc-0.far'},
    ])
    self.assertEqual(packages, {os.path.join(run_tests.OUT_DIR, 'abc.far')})
    self.assertTrue(os.path.islink(os.path.join(run_tests.OUT_DIR, 'abc.far')))

  def test_build_test_cases_with_arguments(self):
    test_cases = run_tests.build_test_cases([
        {'test_command': 'test run abc'},
        {'test_command': 'test run def -- --args'},
    ])
    self.assertEqual(
        test_cases,
        [run_tests.TestCase(package='abc'),
         run_tests.TestCase(package='def', args='--args')]
    )


if __name__ == '__main__':
  try:
    unittest.main()
  finally:
    # Clean up the temporary files. Since it's in /tmp/, ignore the errors.
    shutil.rmtree(run_tests.OUT_DIR, ignore_errors=True)
