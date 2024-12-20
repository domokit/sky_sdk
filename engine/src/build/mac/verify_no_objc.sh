#!/bin/bash

# Copyright (c) 2011 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# This script makes sure that no __OBJC,__image_info section appears in the
# executable file built by the Xcode target that runs the script. If such a
# section appears, the script prints an error message and exits nonzero.
#
# Why is this important?
#
# On 10.5, there's a bug in CFBundlePreflightExecutable that causes it to
# crash when operating in an executable that has not loaded at its default
# address (that is, when it's a position-independent executable with the
# MH_PIE bit set in its mach_header) and the executable has an
# __OBJC,__image_info section. See http://crbug.com/88697.
#
# Chrome's main executables don't use any Objective-C at all, and don't need
# to carry this section around. Not linking them as Objective-C when they
# don't need it anyway saves about 4kB in the linked executable, although most
# of that 4kB is just filled with zeroes.
#
# This script makes sure that nobody goofs and accidentally introduces these
# sections into the main executables.

set -eu

executable="${BUILT_PRODUCTS_DIR}/${EXECUTABLE_PATH}"

if xcrun otool -arch i386 -o "${executable}" | grep -q '^Contents.*section$'; \
then
  echo "${0}: ${executable} has an __OBJC,__image_info section" 2>&1
  exit 1
fi

if [[ ${PIPESTATUS[0]} -ne 0 ]]; then
  echo "${0}: otool failed" 2>&1
  exit 1
fi

exit 0
