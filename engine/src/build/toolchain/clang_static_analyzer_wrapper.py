#!/usr/bin/env python3
#
# Copyright 2017 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
"""Adds an analysis build step to invocations of the Clang C/C++ compiler.
Usage: clang_static_analyzer_wrapper.py <compiler> [args...]
"""
import argparse
import fnmatch
import itertools
import os
import sys
import wrapper_utils
# Flags used to enable analysis for Clang invocations.
analyzer_enable_flags = [
    '--analyze',
]
# Flags used to configure the analyzer's behavior.
analyzer_option_flags = [
    '-fdiagnostics-show-option',
    '-analyzer-checker=cplusplus',
    '-analyzer-opt-analyze-nested-blocks',
    '-analyzer-output=text',
    '-analyzer-config',
    'suppress-c++-stdlib=true',
# List of checkers to execute.
# The full list of checkers can be found at
# https://clang-analyzer.llvm.org/available_checks.html.
    '-analyzer-checker=core',
    '-analyzer-checker=unix',
    '-analyzer-checker=deadcode',
]
# Prepends every element of a list |args| with |token|.
# e.g. ['-analyzer-foo', '-analyzer-bar'] => ['-Xanalyzer', '-analyzer-foo',
#                                             '-Xanalyzer', '-analyzer-bar']
def interleave_args(args, token):
  return list(sum(list(zip([token] * len(args), args)), ()))
def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--mode',
                      choices=['clang', 'cl'],
                      required=True,
                      help='Specifies the compiler argument convention to use.')
  parser.add_argument('args', nargs=argparse.REMAINDER)
  parsed_args = parser.parse_args()
  prefix = '-Xclang' if parsed_args.mode == 'cl' else '-Xanalyzer'
  cmd = parsed_args.args + analyzer_enable_flags + \
        interleave_args(analyzer_option_flags, prefix)
  returncode, stderr = wrapper_utils.CaptureCommandStderr(
      wrapper_utils.CommandToRun(cmd))
  sys.stderr.write(stderr.decode())
  returncode, stderr = wrapper_utils.CaptureCommandStderr(
    wrapper_utils.CommandToRun(parsed_args.args))
  sys.stderr.write(stderr.decode())
  return returncode
if __name__ == '__main__':
  sys.exit(main())
