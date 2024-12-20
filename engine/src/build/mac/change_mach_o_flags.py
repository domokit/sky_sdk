#!/usr/bin/env python3
#
# Copyright (c) 2011 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Usage: change_mach_o_flags.py [--executable-heap] [--no-pie] <executablepath>

Arranges for the executable at |executable_path| to have its data (heap)
pages protected to prevent execution on Mac OS X 10.7 ("Lion"), and to have
the PIE (position independent executable) bit set to enable ASLR (address
space layout randomization). With --executable-heap or --no-pie, the
respective bits are cleared instead of set, making the heap executable or
disabling PIE/ASLR.

This script is able to operate on thin (single-architecture) Mach-O files
and fat (universal, multi-architecture) files. When operating on fat files,
it will set or clear the bits for each architecture contained therein.

NON-EXECUTABLE HEAP

Traditionally in Mac OS X, 32-bit processes did not have data pages set to
prohibit execution. Although user programs could call mprotect and
mach_vm_protect to deny execution of code in data pages, the kernel would
silently ignore such requests without updating the page tables, and the
hardware would happily execute code on such pages. 64-bit processes were
always given proper hardware protection of data pages. This behavior was
controllable on a system-wide level via the vm.allow_data_exec sysctl, which
is set by default to 1. The bit with value 1 (set by default) allows code
execution on data pages for 32-bit processes, and the bit with value 2
(clear by default) does the same for 64-bit processes.

In Mac OS X 10.7, executables can "opt in" to having hardware protection
against code execution on data pages applied. This is done by setting a new
bit in the |flags| field of an executable's |mach_header|. When
MH_NO_HEAP_EXECUTION is set, proper protections will be applied, regardless
of the setting of vm.allow_data_exec. See xnu-1699.22.73/osfmk/vm/vm_map.c
override_nx and xnu-1699.22.73/bsd/kern/mach_loader.c load_machfile.

The Apple toolchain has been revised to set the MH_NO_HEAP_EXECUTION when
producing executables, provided that -allow_heap_execute is not specified
at link time. Only linkers shipping with Xcode 4.0 and later (ld64-123.2 and
later) have this ability. See ld64-123.2.1/src/ld/Options.cpp
Options::reconfigureDefaults() and
ld64-123.2.1/src/ld/HeaderAndLoadCommands.hpp
HeaderAndLoadCommandsAtom<A>::flags().

This script sets the MH_NO_HEAP_EXECUTION bit on Mach-O executables. It is
intended for use with executables produced by a linker that predates Apple's
modifications to set this bit itself. It is also useful for setting this bit
for non-i386 executables, including x86_64 executables. Apple's linker only
sets it for 32-bit i386 executables, presumably under the assumption that
the value of vm.allow_data_exec is set in stone. However, if someone were to
change vm.allow_data_exec to 2 or 3, 64-bit x86_64 executables would run
without hardware protection against code execution on data pages. This
script can set the bit for x86_64 executables, guaranteeing that they run
with appropriate protection even when vm.allow_data_exec has been tampered
with.

POSITION-INDEPENDENT EXECUTABLES/ADDRESS SPACE LAYOUT RANDOMIZATION

This script sets or clears the MH_PIE bit in an executable's Mach-O header,
enabling or disabling position independence on Mac OS X 10.5 and later.
Processes running position-independent executables have varying levels of
ASLR protection depending on the OS release. The main executable's load
address, shared library load addresess, and the heap and stack base
addresses may be randomized. Position-independent executables are produced
by supplying the -pie flag to the linker (or defeated by supplying -no_pie).
Executables linked with a deployment target of 10.7 or higher have PIE on
by default.

This script is never strictly needed during the build to enable PIE, as all
linkers used are recent enough to support -pie. However, it's used to
disable the PIE bit as needed on already-linked executables.
"""

import optparse
import os
import struct
import sys


# <mach-o/fat.h>
FAT_MAGIC = 0xcafebabe
FAT_CIGAM = 0xbebafeca

# <mach-o/loader.h>
MH_MAGIC = 0xfeedface
MH_CIGAM = 0xcefaedfe
MH_MAGIC_64 = 0xfeedfacf
MH_CIGAM_64 = 0xcffaedfe
MH_EXECUTE = 0x2
MH_PIE = 0x00200000
MH_NO_HEAP_EXECUTION = 0x01000000


class MachOError(Exception):
  """A class for exceptions thrown by this module."""

  pass


def CheckedSeek(file, offset):
  """Seeks the file-like object at |file| to offset |offset| and raises a
  MachOError if anything funny happens."""

  file.seek(offset, os.SEEK_SET)
  new_offset = file.tell()
  if new_offset != offset:
    raise MachOError('seek: expected offset %d, observed %d' % (offset, new_offset))


def CheckedRead(file, count):
  """Reads |count| bytes from the file-like |file| object, raising a
  MachOError if any other number of bytes is read."""

  bytes = file.read(count)
  if len(bytes) != count:
    raise MachOError('read: expected length %d, observed %d' % (count, len(bytes)))

  return bytes


def ReadUInt32(file, endian):
  """Reads an unsinged 32-bit integer from the file-like |file| object,
  treating it as having endianness specified by |endian| (per the |struct|
  module), and returns it as a number. Raises a MachOError if the proper
  length of data can't be read from |file|."""

  bytes = CheckedRead(file, 4)

  (uint32,) = struct.unpack(endian + 'I', bytes)
  return uint32


def ReadMachHeader(file, endian):
  """Reads an entire |mach_header| structure (<mach-o/loader.h>) from the
  file-like |file| object, treating it as having endianness specified by
  |endian| (per the |struct| module), and returns a 7-tuple of its members
  as numbers. Raises a MachOError if the proper length of data can't be read
  from |file|."""

  bytes = CheckedRead(file, 28)

  magic, cputype, cpusubtype, filetype, ncmds, sizeofcmds, flags = \
      struct.unpack(endian + '7I', bytes)
  return magic, cputype, cpusubtype, filetype, ncmds, sizeofcmds, flags


def ReadFatArch(file):
  """Reads an entire |fat_arch| structure (<mach-o/fat.h>) from the file-like
  |file| object, treating it as having endianness specified by |endian|
  (per the |struct| module), and returns a 5-tuple of its members as numbers.
  Raises a MachOError if the proper length of data can't be read from
  |file|."""

  bytes = CheckedRead(file, 20)

  cputype, cpusubtype, offset, size, align = struct.unpack('>5I', bytes)
  return cputype, cpusubtype, offset, size, align


def WriteUInt32(file, uint32, endian):
  """Writes |uint32| as an unsinged 32-bit integer to the file-like |file|
  object, treating it as having endianness specified by |endian| (per the
  |struct| module)."""

  bytes = struct.pack(endian + 'I', uint32)
  assert len(bytes) == 4

  file.write(bytes)


def HandleMachOFile(file, options, offset=0):
  """Seeks the file-like |file| object to |offset|, reads its |mach_header|,
  and rewrites the header's |flags| field if appropriate. The header's
  endianness is detected. Both 32-bit and 64-bit Mach-O headers are supported
  (mach_header and mach_header_64). Raises MachOError if used on a header that
  does not have a known magic number or is not of type MH_EXECUTE. The
  MH_PIE and MH_NO_HEAP_EXECUTION bits are set or cleared in the |flags| field
  according to |options| and written to |file| if any changes need to be made.
  If already set or clear as specified by |options|, nothing is written."""

  CheckedSeek(file, offset)
  magic = ReadUInt32(file, '<')
  if magic == MH_MAGIC or magic == MH_MAGIC_64:
    endian = '<'
  elif magic == MH_CIGAM or magic == MH_CIGAM_64:
    endian = '>'
  else:
    raise MachOError('Mach-O file at offset %d has illusion of magic' % offset)

  CheckedSeek(file, offset)
  magic, cputype, cpusubtype, filetype, ncmds, sizeofcmds, flags = \
      ReadMachHeader(file, endian)
  assert magic == MH_MAGIC or magic == MH_MAGIC_64
  if filetype != MH_EXECUTE:
    raise MachOError('Mach-O file at offset %d is type 0x%x, expected MH_EXECUTE' % \
              (offset, filetype))

  original_flags = flags

  if options.no_heap_execution:
    flags |= MH_NO_HEAP_EXECUTION
  else:
    flags &= ~MH_NO_HEAP_EXECUTION

  if options.pie:
    flags |= MH_PIE
  else:
    flags &= ~MH_PIE

  if flags != original_flags:
    CheckedSeek(file, offset + 24)
    WriteUInt32(file, flags, endian)


def HandleFatFile(file, options, fat_offset=0):
  """Seeks the file-like |file| object to |offset| and loops over its
  |fat_header| entries, calling HandleMachOFile for each."""

  CheckedSeek(file, fat_offset)
  magic = ReadUInt32(file, '>')
  assert magic == FAT_MAGIC

  nfat_arch = ReadUInt32(file, '>')

  for index in range(0, nfat_arch):
    cputype, cpusubtype, offset, size, align = ReadFatArch(file)
    assert size >= 28

    # HandleMachOFile will seek around. Come back here after calling it, in
    # case it sought.
    fat_arch_offset = file.tell()
    HandleMachOFile(file, options, offset)
    CheckedSeek(file, fat_arch_offset)


def main(me, args):
  parser = optparse.OptionParser('%prog [options] <executable_path>')
  parser.add_option('--executable-heap', action='store_false',
                    dest='no_heap_execution', default=True,
                    help='Clear the MH_NO_HEAP_EXECUTION bit')
  parser.add_option('--no-pie', action='store_false',
                    dest='pie', default=True,
                    help='Clear the MH_PIE bit')
  (options, loose_args) = parser.parse_args(args)
  if len(loose_args) != 1:
    parser.print_usage()
    return 1

  executable_path = loose_args[0]
  executable_file = open(executable_path, 'rb+')

  magic = ReadUInt32(executable_file, '<')
  if magic == FAT_CIGAM:
    # Check FAT_CIGAM and not FAT_MAGIC because the read was little-endian.
    HandleFatFile(executable_file, options)
  elif magic == MH_MAGIC or magic == MH_CIGAM or \
      magic == MH_MAGIC_64 or magic == MH_CIGAM_64:
    HandleMachOFile(executable_file, options)
  else:
    raise MachOError('%s is not a Mach-O or fat file' % executable_file)

  executable_file.close()
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[0], sys.argv[1:]))
