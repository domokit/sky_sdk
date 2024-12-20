#!/bin/sh

# Copyright (c) 2011 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# This is a small wrapper script around change_mach_o_flags.py allowing it to
# be invoked easily from Xcode. change_mach_o_flags.py expects its arguments
# on the command line, but Xcode puts its parameters in the environment.

set -e

exec "$(dirname "${0}")/change_mach_o_flags.py" \
     "${@}" \
     "${BUILT_PRODUCTS_DIR}/${EXECUTABLE_PATH}"
