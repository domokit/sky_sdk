#!/usr/bin/env python3
#
# Copyright (c) 2013 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.



import json
import os
import subprocess
import sys
import re
from optparse import OptionParser

# This script runs pkg-config, optionally filtering out some results, and
# returns the result.
#
# The result will be [ <includes>, <cflags>, <libs>, <lib_dirs>, <ldflags> ]
# where each member is itself a list of strings.
#
# You can filter out matches using "-v <regexp>" where all results from
# pkgconfig matching the given regular expression will be ignored. You can
# specify more than one regular expression my specifying "-v" more than once.
#
# You can specify a sysroot using "-s <sysroot>" where sysroot is the absolute
# system path to the sysroot used for compiling. This script will attempt to
# generate correct paths for the sysroot.
#
# When using a sysroot, you must also specify the architecture via
# "-a <arch>" where arch is either "x86" or "x64".
#
# CrOS systemroots place pkgconfig files at <systemroot>/usr/share/pkgconfig
# and one of <systemroot>/usr/lib/pkgconfig or <systemroot>/usr/lib64/pkgconfig
# depending on whether the systemroot is for a 32 or 64 bit architecture. They
# specify the 'lib' or 'lib64' of the pkgconfig path by defining the
# 'system_libdir' variable in the args.gn file. pkg_config.gni communicates this
# variable to this script with the "--system_libdir <system_libdir>" flag. If no
# flag is provided, then pkgconfig files are assumed to come from
# <systemroot>/usr/lib/pkgconfig.
#
# Additionally, you can specify the option --atleast-version. This will skip
# the normal outputting of a dictionary and instead print true or false,
# depending on the return value of pkg-config for the given package.


def SetConfigPath(options):
  """Set the PKG_CONFIG_LIBDIR environment variable.

  This takes into account any sysroot and architecture specification from the
  options on the given command line.
  """

  sysroot = options.sysroot
  assert sysroot

  # Compute the library path name based on the architecture.
  arch = options.arch
  if sysroot and not arch:
    print("You must specify an architecture via -a if using a sysroot.")
    sys.exit(1)

  libdir = sysroot + '/usr/' + options.system_libdir + '/pkgconfig'
  libdir += ':' + sysroot + '/usr/share/pkgconfig'
  os.environ['PKG_CONFIG_LIBDIR'] = libdir
  return libdir


def GetPkgConfigPrefixToStrip(options, args):
  """Returns the prefix from pkg-config where packages are installed.

  This returned prefix is the one that should be stripped from the beginning of
  directory names to take into account sysroots.
  """
  # Some sysroots, like the Chromium OS ones, may generate paths that are not
  # relative to the sysroot. For example,
  # /path/to/chroot/build/x86-generic/usr/lib/pkgconfig/pkg.pc may have all
  # paths relative to /path/to/chroot (i.e. prefix=/build/x86-generic/usr)
  # instead of relative to /path/to/chroot/build/x86-generic (i.e prefix=/usr).
  # To support this correctly, it's necessary to extract the prefix to strip
  # from pkg-config's |prefix| variable.
  prefix = subprocess.check_output([options.pkg_config,
      "--variable=prefix"] + args, env=os.environ).decode('utf-8')
  if prefix[-4] == '/usr':
    return prefix[4:]
  return prefix


def MatchesAnyRegexp(flag, list_of_regexps):
  """Returns true if the first argument matches any regular expression in the
  given list."""
  for regexp in list_of_regexps:
    if regexp.search(flag) != None:
      return True
  return False


def RewritePath(path, strip_prefix, sysroot):
  """Rewrites a path by stripping the prefix and prepending the sysroot."""
  if os.path.isabs(path) and not path.startswith(sysroot):
    if path.startswith(strip_prefix):
      path = path[len(strip_prefix):]
    path = path.lstrip('/')
    return os.path.join(sysroot, path)
  else:
    return path


def main():
  # If this is run on non-Linux platforms, just return nothing and indicate
  # success. This allows us to "kind of emulate" a Linux build from other
  # platforms.
  if "linux" not in sys.platform:
    print("[[],[],[],[],[]]")
    return 0

  parser = OptionParser()
  parser.add_option('-d', '--debug', action='store_true')
  parser.add_option('-p', action='store', dest='pkg_config', type='string',
                    default='pkg-config')
  parser.add_option('-v', action='append', dest='strip_out', type='string')
  parser.add_option('-s', action='store', dest='sysroot', type='string')
  parser.add_option('-a', action='store', dest='arch', type='string')
  parser.add_option('--system_libdir', action='store', dest='system_libdir',
                    type='string', default='lib')
  parser.add_option('--atleast-version', action='store',
                    dest='atleast_version', type='string')
  parser.add_option('--libdir', action='store_true', dest='libdir')
  parser.add_option('--dridriverdir', action='store_true', dest='dridriverdir')
  parser.add_option('--version-as-components', action='store_true',
                    dest='version_as_components')
  (options, args) = parser.parse_args()

  # Make a list of regular expressions to strip out.
  strip_out = []
  if options.strip_out != None:
    for regexp in options.strip_out:
      strip_out.append(re.compile(regexp))

  if options.sysroot:
    libdir = SetConfigPath(options)
    if options.debug:
      sys.stderr.write('PKG_CONFIG_LIBDIR=%s\n' % libdir)
    prefix = GetPkgConfigPrefixToStrip(options, args)
  else:
    prefix = ''

  if options.atleast_version:
    # When asking for the return value, just run pkg-config and print the return
    # value, no need to do other work.
    if not subprocess.call([options.pkg_config,
                            "--atleast-version=" + options.atleast_version] +
                            args):
      print("true")
    else:
      print("false")
    return 0

  if options.version_as_components:
    cmd = [options.pkg_config, "--modversion"] + args
    try:
      version_string = subprocess.check_output(cmd).decode('utf-8')
    except:
      sys.stderr.write('Error from pkg-config.\n')
      return 1
    print(json.dumps(list(map(int, version_string.strip().split(".")))))
    return 0


  if options.libdir:
    cmd = [options.pkg_config, "--variable=libdir"] + args
    if options.debug:
      sys.stderr.write('Running: %s\n' % cmd)
    try:
      libdir = subprocess.check_output(cmd).decode('utf-8')
    except:
      print("Error from pkg-config.")
      return 1
    sys.stdout.write(libdir.strip())
    return 0

  if options.dridriverdir:
    cmd = [options.pkg_config, "--variable=dridriverdir"] + args
    if options.debug:
      sys.stderr.write('Running: %s\n' % cmd)
    try:
      dridriverdir = subprocess.check_output(cmd).decode('utf-8')
    except:
      print("Error from pkg-config.")
      return 1
    sys.stdout.write(dridriverdir.strip())
    return

  cmd = [options.pkg_config, "--cflags", "--libs"] + args
  if options.debug:
    sys.stderr.write('Running: %s\n' % ' '.join(cmd))

  try:
    flag_string = subprocess.check_output(cmd).decode('utf-8')
  except:
    sys.stderr.write('Could not run pkg-config.\n')
    return 1

  # For now just split on spaces to get the args out. This will break if
  # pkgconfig returns quoted things with spaces in them, but that doesn't seem
  # to happen in practice.
  all_flags = flag_string.strip().split(' ')


  sysroot = options.sysroot
  if not sysroot:
    sysroot = ''

  includes = []
  cflags = []
  libs = []
  lib_dirs = []

  for flag in all_flags[:]:
    if len(flag) == 0 or MatchesAnyRegexp(flag, strip_out):
      continue;

    if flag[:2] == '-l':
      libs.append(RewritePath(flag[2:], prefix, sysroot))
    elif flag[:2] == '-L':
      lib_dirs.append(RewritePath(flag[2:], prefix, sysroot))
    elif flag[:2] == '-I':
      includes.append(RewritePath(flag[2:], prefix, sysroot))
    elif flag[:3] == '-Wl':
      # Don't allow libraries to control ld flags.  These should be specified
      # only in build files.
      pass
    elif flag == '-pthread':
      # Many libs specify "-pthread" which we don't need since we always include
      # this anyway. Removing it here prevents a bunch of duplicate inclusions
      # on the command line.
      pass
    else:
      cflags.append(flag)

  # Output a GN array, the first one is the cflags, the second are the libs. The
  # JSON formatter prints GN compatible lists when everything is a list of
  # strings.
  print(json.dumps([includes, cflags, libs, lib_dirs]))
  return 0


if __name__ == '__main__':
  sys.exit(main())
