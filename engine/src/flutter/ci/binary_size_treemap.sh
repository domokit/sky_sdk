#!/bin/bash
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# Run a tool that generates a treemap showing the contribution of each
# component to the size of a binary.
#
# Usage:
#   binary_size_treemap.sh [binary_path] [output_dir]

set -e

INPUT_PATH="$(cd $(dirname "$1"); pwd -P)/$(basename "$1")"
DEST_DIR="$(cd $(dirname "$2"); pwd -P)/$(basename "$2")"
CI_DIRECTORY=$(cd $(dirname "${BASH_SOURCE[0]}"); pwd -P)
ENGINE_BUILDROOT=$(cd "$CI_DIRECTORY/../.."; pwd -P)

if [ "$(uname)" == "Darwin" ]; then
  NDK_PLATFORM="darwin-x86_64"
else
  NDK_PLATFORM="linux-x86_64"
fi
ADDR2LINE="flutter/third_party/android_tools/ndk/toolchains/llvm/prebuilt/$NDK_PLATFORM/bin/llvm-addr2line"
NM="flutter/third_party/android_tools/ndk/toolchains/llvm/prebuilt/$NDK_PLATFORM/bin/llvm-nm"

# Run the binary size script from the buildroot directory so the treemap path
# navigation will start from there.
cd "$ENGINE_BUILDROOT"
RUN_BINARY_SIZE_ANALYSIS="flutter/third_party/dart/third_party/binary_size/src/run_binary_size_analysis.py"
python3 "$RUN_BINARY_SIZE_ANALYSIS" --library "$INPUT_PATH" --destdir "$DEST_DIR" --addr2line-binary "$ADDR2LINE" --nm-binary "$NM" --jobs 1 --no-check-support
