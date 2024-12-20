# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import imp
import os
import unittest

SKY_TOOLS = os.path.dirname(os.path.abspath(__file__))
gn = imp.load_source('gn', os.path.join(SKY_TOOLS, 'gn'))


class GNTestCase(unittest.TestCase):

  def _expect_build_dir(self, arg_list, expected_build_dir):
    args = gn.parse_args(['gn'] + arg_list)
    self.assertEqual(gn.get_out_dir(args), expected_build_dir)

  def test_get_out_dir(self):
    self._expect_build_dir(['--runtime-mode', 'debug'], os.path.join('out', 'host_debug'))
    self._expect_build_dir(['--runtime-mode', 'release'], os.path.join('out', 'host_release'))
    self._expect_build_dir(['--ios'], os.path.join('out', 'ios_debug'))
    self._expect_build_dir(['--ios', '--darwin-extension-safe'],
                           os.path.join('out', 'ios_debug_extension_safe'))
    self._expect_build_dir(['--ios', '--runtime-mode', 'release'],
                           os.path.join('out', 'ios_release'))
    self._expect_build_dir(['--ios', '--darwin-extension-safe', '--runtime-mode', 'release'],
                           os.path.join('out', 'ios_release_extension_safe'))
    self._expect_build_dir(['--android'], os.path.join('out', 'android_debug'))
    self._expect_build_dir(['--android', '--runtime-mode', 'release'],
                           os.path.join('out', 'android_release'))

  def _gn_args(self, arg_list):
    args = gn.parse_args(['gn'] + arg_list)
    return gn.to_gn_args(args)

  def test_to_gn_args(self):
    # This would not necessarily be true on a 32-bit machine?
    self.assertEqual(
        self._gn_args(['--ios', '--simulator', '--simulator-cpu', 'x64'])['target_cpu'], 'x64'
    )
    self.assertEqual(self._gn_args(['--ios'])['target_cpu'], 'arm64')

  def test_cannot_use_android_and_enable_unittests(self):
    with self.assertRaises(Exception):
      self._gn_args(['--android', '--enable-unittests'])

  def test_cannot_use_ios_and_enable_unittests(self):
    with self.assertRaises(Exception):
      self._gn_args(['--ios', '--enable-unittests'])

  def test_parse_size(self):
    self.assertEqual(gn.parse_size('5B'), 5)
    self.assertEqual(gn.parse_size('5KB'), 5 * 2**10)
    self.assertEqual(gn.parse_size('5MB'), 5 * 2**20)
    self.assertEqual(gn.parse_size('5GB'), 5 * 2**30)


if __name__ == '__main__':
  unittest.main()
