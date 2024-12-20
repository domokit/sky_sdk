#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

'''
Tests for font-subset
'''

import argparse
import filecmp
import os
import subprocess
import sys
from zipfile import ZipFile

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
MATERIAL_TTF = os.path.join(SCRIPT_DIR, 'fixtures', 'MaterialIcons-Regular.ttf')
VARIABLE_MATERIAL_TTF = os.path.join(SCRIPT_DIR, 'fixtures', 'MaterialSymbols-Variable.ttf')

COMPARE_TESTS = (
    (True, '1.ttf', MATERIAL_TTF, [r'57347']),
    (True, '1.ttf', MATERIAL_TTF, [r'0xE003']),
    (True, '1.ttf', MATERIAL_TTF, [r'\uE003']),
    (False, '1.ttf', MATERIAL_TTF, [r'57348']),  # False because different codepoint
    (True, '2.ttf', MATERIAL_TTF, [r'0xE003', r'0xE004']),
    (True, '2.ttf', MATERIAL_TTF, [r'0xE003',
                                   r'optional:0xE004']),  # Optional codepoint that is found
    (True, '2.ttf', MATERIAL_TTF, [
        r'0xE003',
        r'0xE004',
        r'optional:0x12',
    ]),  # Optional codepoint that is not found
    (True, '2.ttf', MATERIAL_TTF, [
        r'0xE003',
        r'0xE004',
        r'57347',
    ]),  # Duplicated codepoint
    (True, '3.ttf', MATERIAL_TTF, [
        r'0xE003',
        r'0xE004',
        r'0xE021',
    ]),
    # repeat tests with variable input font and verified variable output goldens
    (True, '1variable.ttf', VARIABLE_MATERIAL_TTF, [r'57347']),
    (True, '1variable.ttf', VARIABLE_MATERIAL_TTF, [r'0xE003']),
    (True, '1variable.ttf', VARIABLE_MATERIAL_TTF, [r'\uE003']),
    (False, '1variable.ttf', VARIABLE_MATERIAL_TTF, [r'57348'
                                                    ]),  # False because different codepoint
    (True, '2variable.ttf', VARIABLE_MATERIAL_TTF, [r'0xE003', r'0xE004']),
    (True, '2variable.ttf', VARIABLE_MATERIAL_TTF, [
        r'0xE003',
        r'0xE004',
        r'57347',
    ]),  # Duplicated codepoint
    (True, '3variable.ttf', VARIABLE_MATERIAL_TTF, [
        r'0xE003',
        r'0xE004',
        r'0xE021',
    ]),
)


def fail_tests(font_subset):
  return [
      ([font_subset, 'output.ttf', 'does-not-exist.ttf'], [
          '1',
      ]),  # non-existent input font
      ([font_subset, 'output.ttf', MATERIAL_TTF], [
          '0xFFFFFFFF',
      ]),  # Value too big.
      ([font_subset, 'output.ttf', MATERIAL_TTF], [
          '-1',
      ]),  # invalid value
      ([font_subset, 'output.ttf', MATERIAL_TTF], [
          'foo',
      ]),  # no valid values
      ([font_subset, 'output.ttf', MATERIAL_TTF], [
          '0xE003',
          '0x12',
          '0xE004',
      ]),  # codepoint not in font
      ([font_subset, 'non-existent-dir/output.ttf', MATERIAL_TTF], [
          '0xE003',
      ]),  # dir doesn't exist
      ([font_subset, 'output.ttf', MATERIAL_TTF], [
          ' ',
      ]),  # empty input
      ([font_subset, 'output.ttf', MATERIAL_TTF], []),  # empty input
      ([font_subset, 'output.ttf', MATERIAL_TTF], ['']),  # empty input
      # repeat tests with variable input font
      ([font_subset, 'output.ttf', VARIABLE_MATERIAL_TTF], [
          '0xFFFFFFFF',
      ]),  # Value too big.
      ([font_subset, 'output.ttf', VARIABLE_MATERIAL_TTF], [
          '-1',
      ]),  # invalid value
      ([font_subset, 'output.ttf', VARIABLE_MATERIAL_TTF], [
          'foo',
      ]),  # no valid values
      ([font_subset, 'output.ttf', VARIABLE_MATERIAL_TTF], [
          '0xE003',
          '0x12',
          '0xE004',
      ]),  # codepoint not in font
      ([font_subset, 'non-existent-dir/output.ttf', VARIABLE_MATERIAL_TTF], [
          '0xE003',
      ]),  # dir doesn't exist
      ([font_subset, 'output.ttf', VARIABLE_MATERIAL_TTF], [
          ' ',
      ]),  # empty input
      ([font_subset, 'output.ttf', VARIABLE_MATERIAL_TTF], []),  # empty input
      ([font_subset, 'output.ttf', VARIABLE_MATERIAL_TTF], ['']),  # empty input
  ]


def run_cmd(cmd, codepoints, fail=False):
  print('Running command:')
  print('       %s' % ' '.join(cmd))
  print('STDIN: "%s"' % ' '.join(codepoints))
  p = subprocess.Popen(
      cmd, stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.PIPE, cwd=SRC_DIR
  )
  stdout_data, stderr_data = p.communicate(input=' '.join(codepoints).encode())
  if p.returncode != 0 and fail == False:
    print('FAILURE: %s' % p.returncode)
    print('STDOUT:')
    print(stdout_data)
    print('STDERR:')
    print(stderr_data)
  elif p.returncode == 0 and fail == True:
    print('FAILURE - test passed but should have failed.')
    print('STDOUT:')
    print(stdout_data)
    print('STDERR:')
    print(stderr_data)
  else:
    print('Success.')

  return p.returncode


def test_zip(font_subset_zip, exe):
  with ZipFile(font_subset_zip, 'r') as zip:
    files = zip.namelist()
    if 'font-subset%s' % exe not in files:
      print('expected %s to contain font-subset%s' % (files, exe))
      return 1
    return 0


# Maps the platform name to the output directory of the font artifacts.
def platform_to_path(os, cpu):
  d = {
      'darwin': 'darwin-',
      'linux': 'linux-',
      'linux2': 'linux-',
      'cygwin': 'windows-',
      'win': 'windows-',
      'win32': 'windows-',
  }
  return d[os] + cpu


def main():
  parser = argparse.ArgumentParser(description='Runs font-subset tests.')
  parser.add_argument('--variant', type=str, required=True)
  parser.add_argument('--target-cpu', type=str, default='x64')
  args = parser.parse_args()
  variant = args.variant

  is_windows = sys.platform.startswith(('cygwin', 'win'))
  exe = '.exe' if is_windows else ''
  font_subset = os.path.join(SRC_DIR, 'out', variant, 'font-subset' + exe)
  font_subset_zip = os.path.join(
      SRC_DIR, 'out', variant, 'zip_archives', platform_to_path(sys.platform, args.target_cpu),
      'font-subset.zip'
  )
  if not os.path.isfile(font_subset):
    raise Exception(
        'Could not locate font-subset%s in %s - build before running this script.' % (exe, variant)
    )

  print('Using font subset binary at %s (%s)' % (font_subset, font_subset_zip))
  failures = 0

  failures += test_zip(font_subset_zip, exe)

  for should_pass, golden_font, input_font, codepoints in COMPARE_TESTS:
    gen_ttf = os.path.join(SCRIPT_DIR, 'gen', golden_font)
    golden_ttf = os.path.join(SCRIPT_DIR, 'fixtures', golden_font)
    cmd = [font_subset, gen_ttf, input_font]
    run_cmd(cmd, codepoints)
    cmp = filecmp.cmp(gen_ttf, golden_ttf, shallow=False)
    if (should_pass and not cmp) or (not should_pass and cmp):
      print('Test case %s failed.' % cmd)
      failures += 1

  with open(os.devnull, 'w') as devnull:
    for cmd, codepoints in fail_tests(font_subset):
      if run_cmd(cmd, codepoints, fail=True) == 0:
        failures += 1

  if failures > 0:
    print('%s test(s) failed.' % failures)
    return 1

  print('All tests passed')
  return 0


if __name__ == '__main__':
  sys.exit(main())
