#!/bin/sh

# Copyright (c) 2012 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# This script is a convenience to run GYP for /src/chrome/chrome_nibs.gyp
# with the Xcode generator (as you likely use ninja). Documentation:
#   http://dev.chromium.org/developers/design-documents/mac-xib-files

set -e

RELSRC=$(dirname "$0")/../..
SRC=$(cd "$RELSRC" && pwd)
export PYTHONPATH="$PYTHONPATH:$SRC/build"
export GYP_GENERATORS=xcode
"$SRC/tools/gyp/gyp" -I"$SRC/build/common.gypi" "$SRC/chrome/chrome_nibs.gyp"
echo "You can now edit XIB files in Xcode using:"
echo "  $SRC/chrome/chrome_nibs.xcodeproj"
