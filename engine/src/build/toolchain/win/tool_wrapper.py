# Copyright (c) 2012 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Utility functions for Windows builds.
This file is copied to the build directory as part of toolchain setup and
is used to set up calls to tools used by the build that need wrappers.
"""

from __future__ import print_function

import os
import re
import shutil
import subprocess
import stat
import sys


BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# A regex matching an argument corresponding to the output filename passed to
# link.exe.
_LINK_EXE_OUT_ARG = re.compile('/OUT:(?P<out>.+)$', re.IGNORECASE)

def main(args):
  exit_code = WinTool().Dispatch(args)
  if exit_code is not None:
    sys.exit(exit_code)


class WinTool(object):
  """This class performs all the Windows tooling steps. The methods can either
  be executed directly, or dispatched from an argument list."""

  def _UseSeparateMspdbsrv(self, env, args):
    """Allows to use a unique instance of mspdbsrv.exe per linker instead of a
    shared one."""
    if len(args) < 1:
      raise Exception("Not enough arguments")

    if args[0] != 'link.exe':
      return

    # Use the output filename passed to the linker to generate an endpoint name
    # for mspdbsrv.exe.
    endpoint_name = None
    for arg in args:
      m = _LINK_EXE_OUT_ARG.match(arg)
      if m:
        endpoint_name = re.sub(r'\W+', '',
            '%s_%d' % (m.group('out'), os.getpid()))
        break

    if endpoint_name is None:
      return

    # Adds the appropriate environment variable. This will be read by link.exe
    # to know which instance of mspdbsrv.exe it should connect to (if it's
    # not set then the default endpoint is used).
    env['_MSPDBSRV_ENDPOINT_'] = endpoint_name

  def Dispatch(self, args):
    """Dispatches a string command to a method."""
    if len(args) < 1:
      raise Exception("Not enough arguments")

    method = "Exec%s" % self._CommandifyName(args[0])
    return getattr(self, method)(*args[1:])

  def _CommandifyName(self, name_string):
    """Transforms a tool name like recursive-mirror to RecursiveMirror."""
    return name_string.title().replace('-', '')

  def _GetEnv(self, arch):
    """Gets the saved environment from a file for a given architecture."""
    # The environment is saved as an "environment block" (see CreateProcess
    # and msvs_emulation for details). We convert to a dict here.
    # Drop last 2 NULs, one for list terminator, one for trailing vs. separator.
    pairs = open(arch).read()[:-2].split('\0')
    kvs = [item.split('=', 1) for item in pairs]
    return dict(kvs)

  def ExecDeleteFile(self, path):
    """Simple file delete command."""
    if os.path.exists(path):
      os.unlink(path)

  def ExecRecursiveMirror(self, source, dest):
    """Emulation of rm -rf out && cp -af in out."""
    if os.path.exists(dest):
      if os.path.isdir(dest):
        def _on_error(fn, path, dummy_excinfo):
          # The operation failed, possibly because the file is set to
          # read-only. If that's why, make it writable and try the op again.
          if not os.access(path, os.W_OK):
            os.chmod(path, stat.S_IWRITE)
          fn(path)
        shutil.rmtree(dest, onerror=_on_error)
      else:
        if not os.access(dest, os.W_OK):
          # Attempt to make the file writable before deleting it.
          os.chmod(dest, stat.S_IWRITE)
        os.unlink(dest)

    if os.path.isdir(source):
      shutil.copytree(source, dest)
    else:
      shutil.copy2(source, dest)
      # Try to diagnose crbug.com/741603
      if not os.path.exists(dest):
        raise Exception("Copying of %s to %s failed" % (source, dest))

  def ExecLinkWrapper(self, arch, use_separate_mspdbsrv, *args):
    """Filter diagnostic output from link that looks like:
    '   Creating library ui.dll.lib and object ui.dll.exp'
    This happens when there are exports from the dll or exe.
    """
    env = self._GetEnv(arch)
    if use_separate_mspdbsrv == 'True':
      self._UseSeparateMspdbsrv(env, args)
    if sys.platform == 'win32':
      args = list(args)  # *args is a tuple by default, which is read-only.
      args[0] = args[0].replace('/', '\\')
    # https://docs.python.org/2/library/subprocess.html:
    # "On Unix with shell=True [...] if args is a sequence, the first item
    # specifies the command string, and any additional items will be treated as
    # additional arguments to the shell itself.  That is to say, Popen does the
    # equivalent of:
    #   Popen(['/bin/sh', '-c', args[0], args[1], ...])"
    # For that reason, since going through the shell doesn't seem necessary on
    # non-Windows don't do that there.
    pe_name = None
    for arg in args:
      m = _LINK_EXE_OUT_ARG.match(arg)
      if m:
        pe_name = m.group('out')
    link = subprocess.Popen(args, shell=sys.platform == 'win32', env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    # Read output one line at a time as it shows up to avoid OOM failures when
    # GBs of output is produced.
    for line in link.stdout:
      if (not line.startswith(b'   Creating library ')
          and not line.startswith(b'Generating code')
          and not line.startswith(b'Finished generating code')):
        print(line)
    return link.wait()

  def ExecAsmWrapper(self, arch, *args):
    """Filter logo banner from invocations of asm.exe."""
    env = self._GetEnv(arch)
    if sys.platform == 'win32':
      # Windows ARM64 uses clang-cl as assembler which has '/' as path
      # separator, convert it to '\\' when running on Windows.
      args = list(args) # *args is a tuple by default, which is read-only
      args[0] = args[0].replace('/', '\\')
    popen = subprocess.Popen(args, shell=True, env=env,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out, _ = popen.communicate()
    for line in out.decode('utf8').splitlines():
      if not line.startswith(' Assembling: '):
        print(line)
    return popen.returncode

  def ExecRcWrapper(self, arch, *args):
    """Filter logo banner from invocations of rc.exe. Older versions of RC
    don't support the /nologo flag."""
    env = self._GetEnv(arch)
    popen = subprocess.Popen(args, shell=True, env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out, _ = popen.communicate()
    for line in out.splitlines():
      if (not line.startswith(b'Microsoft (R) Windows (R) Resource Compiler') and
          not line.startswith(b'Copyright (C) Microsoft Corporation') and line):
        print(line)
    return popen.returncode

  def ExecActionWrapper(self, arch, rspfile, *dirname):
    """Runs an action command line from a response file using the environment
    for |arch|. If |dirname| is supplied, use that as the working directory."""
    env = self._GetEnv(arch)
    # TODO(scottmg): This is a temporary hack to get some specific variables
    # through to actions that are set after GN-time. http://crbug.com/333738.
    for k, v in os.environ.items():
      if k not in env:
        env[k] = v
    args = open(rspfile).read()
    dirname = dirname[0] if dirname else None
    return subprocess.call(args, shell=True, env=env, cwd=dirname)


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
