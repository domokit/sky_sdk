#!/usr/bin/env python3
#
# Copyright (c) 2012 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Compiler version checking tool for gcc

Print gcc version as XY if you are running gcc X.Y.*.
This is used to tweak build flags for gcc 4.4.
"""

import os
import re
import subprocess
import sys


compiler_version_cache = {}  # Map from (compiler, tool) -> version.


def Usage(program_name):
  print('%s MODE TOOL' % os.path.basename(program_name))
  print('MODE: host or target.')
  print('TOOL: assembler or compiler or linker.')
  return 1


def ParseArgs(args):
  if len(args) != 2:
    raise Exception('Invalid number of arguments')
  mode = args[0]
  tool = args[1]
  if mode not in ('host', 'target'):
    raise Exception('Invalid mode: %s' % mode)
  if tool not in ('assembler', 'compiler', 'linker'):
    raise Exception('Invalid tool: %s' % tool)
  return mode, tool


def GetEnvironFallback(var_list, default):
  """Look up an environment variable from a possible list of variable names."""
  for var in var_list:
    if var in os.environ:
      return os.environ[var]
  return default


def GetVersion(compiler, tool):
  tool_output = tool_error = None
  cache_key = (compiler, tool)
  cached_version = compiler_version_cache.get(cache_key)
  if cached_version:
    return cached_version
  try:
    # Note that compiler could be something tricky like "distcc g++".
    if tool == "compiler":
      compiler = compiler + " -dumpversion"
      # 4.6
      version_re = re.compile(r"(\d+)\.(\d+)")
    elif tool == "assembler":
      compiler = compiler + " -Xassembler --version -x assembler -c /dev/null"
      # Unmodified: GNU assembler (GNU Binutils) 2.24
      # Ubuntu: GNU assembler (GNU Binutils for Ubuntu) 2.22
      # Fedora: GNU assembler version 2.23.2
      version_re = re.compile(r"^GNU [^ ]+ .* (\d+).(\d+).*?$", re.M)
    elif tool == "linker":
      compiler = compiler + " -Xlinker --version"
      # Using BFD linker
      # Unmodified: GNU ld (GNU Binutils) 2.24
      # Ubuntu: GNU ld (GNU Binutils for Ubuntu) 2.22
      # Fedora: GNU ld version 2.23.2
      # Using Gold linker
      # Unmodified: GNU gold (GNU Binutils 2.24) 1.11
      # Ubuntu: GNU gold (GNU Binutils for Ubuntu 2.22) 1.11
      # Fedora: GNU gold (version 2.23.2) 1.11
      version_re = re.compile(r"^GNU [^ ]+ .* (\d+).(\d+).*?$", re.M)
    else:
      raise Exception("Unknown tool %s" % tool)

    # Force the locale to C otherwise the version string could be localized
    # making regex matching fail.
    env = os.environ.copy()
    env["LC_ALL"] = "C"
    pipe = subprocess.Popen(compiler, shell=True, env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    tool_output, tool_error = pipe.communicate()
    if pipe.returncode:
      raise subprocess.CalledProcessError(pipe.returncode, compiler)

    parsed_output = version_re.match(tool_output)
    result = parsed_output.group(1) + parsed_output.group(2)
    compiler_version_cache[cache_key] = result
    return result
  except Exception as e:
    if tool_error:
      sys.stderr.write(tool_error)
    print("compiler_version.py failed to execute:", compiler, file=sys.stderr)
    print(e, file=sys.stderr)
    return ""


def main(args):
  try:
    (mode, tool) = ParseArgs(args[1:])
  except Exception as e:
    sys.stderr.write(e.message + '\n\n')
    return Usage(args[0])

  ret_code, result = ExtractVersion(mode, tool)
  if ret_code == 0:
    print(result)
  return ret_code


def DoMain(args):
  """Hook to be called from gyp without starting a separate python
  interpreter."""
  (mode, tool) = ParseArgs(args)
  ret_code, result = ExtractVersion(mode, tool)
  if ret_code == 0:
    return result
  raise Exception("Failed to extract compiler version for args: %s" % args)


def ExtractVersion(mode, tool):
  # Check if various CXX environment variables exist and use them if they
  # exist. The preferences and fallback order is a close approximation of
  # GenerateOutputForConfig() in GYP's ninja generator.
  # The main difference being not supporting GYP's make_global_settings.
  environments = ['CXX_target', 'CXX']
  if mode == 'host':
    environments = ['CXX_host'] + environments;
  compiler = GetEnvironFallback(environments, 'c++')

  if compiler:
    compiler_version = GetVersion(compiler, tool)
    if compiler_version != "":
      return (0, compiler_version)
  return (1, None)


if __name__ == "__main__":
  sys.exit(main(sys.argv))
