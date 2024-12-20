#!/bin/bash

# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

set -e

# Needed because if it is set, cd may print the path it changed to.
unset CDPATH

# On Mac OS, readlink -f doesn't work, so follow_links traverses the path one
# link at a time, and then cds into the link destination and find out where it
# ends up.
#
# The function is enclosed in a subshell to avoid changing the working directory
# of the caller.
function follow_links() (
  cd -P "$(dirname -- "$1")"
  file="$PWD/$(basename -- "$1")"
  while [[ -L "$file" ]]; do
    cd -P "$(dirname -- "$file")"
    file="$(readlink -- "$file")"
    cd -P "$(dirname -- "$file")"
    file="$PWD/$(basename -- "$file")"
  done
  echo "$file"
)

SCRIPT_DIR=$(follow_links "$(dirname -- "${BASH_SOURCE[0]}")")
ENGINE_DIR="$(cd "$SCRIPT_DIR/../../.."; pwd -P)"

# Find the Dart executable.
case "$(uname -s)" in
  Linux)
    OS="linux"
    ;;
  Darwin)
    OS="macos"
    ;;
  *)
    echo "The host platform is not supported by this tool"
    exit 1
esac

case "$(uname -m)" in
  arm64)
    CPU="arm64"
    ;;
  x86_64)
    CPU="x64"
    ;;
  *)
    echo "The host platform is not supported by this tool"
    exit 1
esac

PLATFORM="${OS}-${CPU}"
DART_SDK_DIR="${ENGINE_DIR}/prebuilts/${PLATFORM}/dart-sdk"
DART="${DART_SDK_DIR}/bin/dart"

# Run the tool indefinitely until there is an error.
COUNT=0
while true; do
  COUNT=$((COUNT + 1))
  echo "Running test iteration $COUNT"
  "$DART" "$SCRIPT_DIR/bin/run_android_tests.dart" "$@"
  # Break if non-zero exit code.
  if [ $? -ne 0 ]; then
    echo "Error running tests. Exiting."
    break
  fi
done
