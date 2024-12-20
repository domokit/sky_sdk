#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import os
import platform
import shutil
import subprocess
import sys


def assert_directory(path, what):
  """Logs an error and exits with EX_NOINPUT if the specified directory doesn't exist."""
  if not os.path.isdir(path):
    log_error('Cannot find %s at %s' % (what, path))
    sys.exit(os.EX_NOINPUT)


def assert_file(path, what):
  """Logs an error and exits with EX_NOINPUT if the specified file doesn't exist."""
  if not os.path.isfile(path):
    log_error('Cannot find %s at %s' % (what, path))
    sys.exit(os.EX_NOINPUT)


def assert_valid_codesign_config(
    framework_dir, zip_contents, entitlements, without_entitlements, unsigned_binaries
):
  """Exits with exit code 1 if the codesign configuration contents are incorrect.
  All Mach-O binaries found within zip_contents exactly must be listed in
  either entitlements or without_entitlements."""
  if _contains_duplicates(entitlements):
    log_error('ERROR: duplicate value(s) found in entitlements.txt')
    log_error_items(sorted(entitlements))
    sys.exit(os.EX_DATAERR)

  if _contains_duplicates(without_entitlements):
    log_error('ERROR: duplicate value(s) found in without_entitlements.txt')
    log_error_items(sorted(without_entitlements))
    sys.exit(os.EX_DATAERR)

  if _contains_duplicates(unsigned_binaries):
    log_error('ERROR: duplicate value(s) found in unsigned_binaries.txt')
    log_error_items(sorted(unsigned_binaries))
    sys.exit(os.EX_DATAERR)

  if _contains_duplicates(entitlements + without_entitlements + unsigned_binaries):
    log_error(
        'ERROR: duplicate value(s) found between '
        'entitlements.txt, without_entitlements.txt, unsigned_binaries.txt'
    )
    log_error_items(sorted(entitlements + without_entitlements + unsigned_binaries))
    sys.exit(os.EX_DATAERR)

  binaries = set()
  for zip_content_path in zip_contents:
    # If file, check if Mach-O binary.
    if _is_macho_binary(os.path.join(framework_dir, zip_content_path)):
      binaries.add(zip_content_path)
    # If directory, check transitive closure of files for Mach-O binaries.
    for root, _, files in os.walk(os.path.join(framework_dir, zip_content_path)):
      for file in [os.path.join(root, f) for f in files]:
        if _is_macho_binary(file):
          binaries.add(os.path.relpath(file, framework_dir))

  # Verify that all Mach-O binaries are listed in either entitlements,
  # without_entitlements, or unsigned_binaries.
  listed_binaries = set(entitlements + without_entitlements + unsigned_binaries)
  if listed_binaries != binaries:
    log_error(
        'ERROR: binaries listed in entitlements.txt, without_entitlements.txt, and'
        'unsigned_binaries.txt do not match the set of binaries in the files to be zipped'
    )
    log_error('Binaries found in files to be zipped:')
    for file in sorted(binaries):
      log_error('    ' + file)

    not_listed = sorted(binaries - listed_binaries)
    if not_listed:
      log_error(
          'Binaries NOT LISTED in entitlements.txt, without_entitlements.txt, '
          'unsigned_binaries.txt:'
      )
      for file in not_listed:
        log_error('    ' + file)

    not_found = sorted(listed_binaries - binaries)
    if not_found:
      log_error(
          'Binaries listed in entitlements.txt, without_entitlements.txt, '
          'unsigned_binaries.txt but NOT FOUND:'
      )
      for file in not_found:
        log_error('    ' + file)
    sys.exit(os.EX_NOINPUT)


def _contains_duplicates(strings):
  """Returns true if the list of strings contains a duplicate value."""
  return len(strings) != len(set(strings))


def _is_macho_binary(filename):
  """Returns True if the specified path is file and a Mach-O binary."""
  if os.path.islink(filename) or not os.path.isfile(filename):
    return False

  with open(filename, 'rb') as file:
    chunk = file.read(4)
    return chunk in (
        b'\xca\xfe\xba\xbe',  # Mach-O Universal Big Endian
        b'\xce\xfa\xed\xfe',  # Mach-O Little Endian (32-bit)
        b'\xcf\xfa\xed\xfe',  # Mach-O Little Endian (64-bit)
        b'\xfe\xed\xfa\xce',  # Mach-O Big Endian (32-bit)
        b'\xfe\xed\xfa\xcf',  # Mach-O Big Endian (64-bit)
    )


def buildroot_relative_path(path):
  """Returns the absolute path to the specified buildroot-relative path."""
  buildroot_dir = os.path.abspath(os.path.join(os.path.realpath(__file__), '..', '..', '..', '..'))
  return os.path.join(buildroot_dir, path)


def copy_binary(source_path, destination_path):
  """Copies a binary, preserving POSIX permissions."""
  assert_file(source_path, 'file to copy')
  shutil.copy2(source_path, destination_path)


def copy_tree(source_path, destination_path, symlinks=False):
  """Performs a recursive copy of a directory. If the destination path is
  present, it is deleted first."""
  assert_directory(source_path, 'directory to copy')
  shutil.rmtree(destination_path, True)
  shutil.copytree(source_path, destination_path, symlinks=symlinks)


def create_fat_macos_framework(args, dst, fat_framework, arm64_framework, x64_framework):
  """Creates a fat framework from two arm64 and x64 frameworks."""
  # Clone the arm64 framework bundle as a starting point.
  copy_tree(arm64_framework, fat_framework, symlinks=True)
  _regenerate_symlinks(fat_framework)
  framework_dylib = get_mac_framework_dylib_path(fat_framework)
  lipo([get_mac_framework_dylib_path(arm64_framework),
        get_mac_framework_dylib_path(x64_framework)], framework_dylib)
  _set_framework_permissions(fat_framework)

  framework_dsym = fat_framework + '.dSYM' if args.dsym else None
  _process_macos_framework(args, dst, framework_dylib, framework_dsym)


def _regenerate_symlinks(framework_dir):
  """Regenerates the framework symlink structure.

  When building on the bots, the framework is produced in one shard, uploaded
  to LUCI's content-addressable storage cache (CAS), then pulled down in
  another shard. When that happens, symlinks are dereferenced, resulting a
  corrupted framework. This regenerates the expected symlink farm.
  """
  # If the dylib is symlinked, assume symlinks are all fine and bail out.
  # The shutil.rmtree calls below only work on directories, and fail on symlinks.
  framework_name = get_framework_name(framework_dir)
  if os.path.islink(os.path.join(framework_dir, framework_name)):
    return

  # Delete any existing files/directories.
  os.remove(os.path.join(framework_dir, framework_name))
  shutil.rmtree(os.path.join(framework_dir, 'Headers'), True)
  shutil.rmtree(os.path.join(framework_dir, 'Modules'), True)
  shutil.rmtree(os.path.join(framework_dir, 'Resources'), True)
  current_version_path = os.path.join(framework_dir, 'Versions', 'Current')
  shutil.rmtree(current_version_path, True)

  # Recreate the expected framework symlinks.
  os.symlink('A', current_version_path)

  os.symlink(
      os.path.join('Versions', 'Current', framework_name),
      os.path.join(framework_dir, framework_name)
  )
  os.symlink(os.path.join('Versions', 'Current', 'Headers'), os.path.join(framework_dir, 'Headers'))
  os.symlink(os.path.join('Versions', 'Current', 'Modules'), os.path.join(framework_dir, 'Modules'))
  os.symlink(
      os.path.join('Versions', 'Current', 'Resources'), os.path.join(framework_dir, 'Resources')
  )


def _set_framework_permissions(framework_dir):
  """Sets framework contents to be world readable, and world executable if user-executable."""
  # Make the framework readable and executable: u=rwx,go=rx.
  subprocess.check_call(['chmod', '755', framework_dir])

  # Add group and other readability to all files.
  versions_path = os.path.join(framework_dir, 'Versions')
  subprocess.check_call(['chmod', '-R', 'og+r', versions_path])
  # Find all the files below the target dir with owner execute permission
  find_subprocess = subprocess.Popen(['find', versions_path, '-perm', '-100', '-print0'],
                                     stdout=subprocess.PIPE)
  # Add execute permission for other and group for all files that had it for owner.
  xargs_subprocess = subprocess.Popen(['xargs', '-0', 'chmod', 'og+x'],
                                      stdin=find_subprocess.stdout)
  find_subprocess.wait()
  xargs_subprocess.wait()


def _process_macos_framework(args, dst, framework_dylib, dsym):
  if dsym:
    extract_dsym(framework_dylib, dsym)

  if args.strip:
    unstripped_out = os.path.join(dst, 'FlutterMacOS.unstripped')
    strip_binary(framework_dylib, unstripped_out)


def create_zip(cwd, zip_filename, paths):
  """Creates a zip archive in cwd, containing a set of cwd-relative files.

  In order to preserve the correct internal structure of macOS frameworks,
  symlinks are preserved (-y). In order to generate reproducible builds,
  owner/group and unix file timestamps are not included in the archive (-X).
  """
  subprocess.check_call(['zip', '-r', '-X', '-y', zip_filename] + paths, cwd=cwd)


def _dsymutil_path():
  """Returns the path to dsymutil within Flutter's clang toolchain."""
  arch_subpath = 'mac-arm64' if platform.processor() == 'arm' else 'mac-x64'
  dsymutil_path = os.path.join('flutter', 'buildtools', arch_subpath, 'clang', 'bin', 'dsymutil')
  return buildroot_relative_path(dsymutil_path)


def get_framework_name(framework_dir):
  """Returns Foo given /path/to/Foo.framework."""
  return os.path.splitext(os.path.basename(framework_dir))[0]


def get_mac_framework_dylib_path(framework_dir):
  """Returns /path/to/Foo.framework/Versions/A/Foo given /path/to/Foo.framework."""
  return os.path.join(framework_dir, 'Versions', 'A', get_framework_name(framework_dir))


def extract_dsym(binary_path, dsym_out_path):
  """Extracts a dSYM bundle from the specified Mach-O binary."""
  arch_dir = 'mac-arm64' if platform.processor() == 'arm' else 'mac-x64'
  dsymutil = buildroot_relative_path(
      os.path.join('flutter', 'buildtools', arch_dir, 'clang', 'bin', 'dsymutil')
  )
  subprocess.check_call([dsymutil, '-o', dsym_out_path, binary_path])


def lipo(input_binaries, output_binary):
  """Uses lipo to create a fat binary from a set of input binaries."""
  subprocess.check_call(['lipo'] + input_binaries + ['-create', '-output', output_binary])


def log_error(message):
  """Writes the message to stderr, followed by a newline."""
  print(message, file=sys.stderr)


def log_error_items(items):
  """Writes each item indented to stderr, followed by a newline."""
  for item in items:
    log_error('  ' + item)


def strip_binary(binary_path, unstripped_copy_path):
  """Makes a copy of an unstripped binary, then strips symbols from the binary."""
  assert_file(binary_path, 'binary to strip')
  shutil.copyfile(binary_path, unstripped_copy_path)
  subprocess.check_call(['strip', '-x', '-S', binary_path])


def write_codesign_config(output_path, paths):
  """Writes an Apple codesign configuration file containing the specified paths."""
  with open(output_path, mode='w', encoding='utf-8') as file:
    if paths:
      file.write('\n'.join(paths) + '\n')
