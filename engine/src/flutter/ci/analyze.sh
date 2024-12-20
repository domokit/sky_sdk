#!/bin/bash
#
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
SRC_DIR="$(cd "$SCRIPT_DIR/../.."; pwd -P)"
FLUTTER_DIR="$SRC_DIR/flutter"

# This shell script takes one optional argument, the path to a dart-sdk/bin
# directory. If not specified, we default to the build output for
# host_debug_unopt.
if [[ $# -eq 0 ]] ; then
  DART_BIN="$SRC_DIR/out/host_debug_unopt/dart-sdk/bin"
  if [[ ! -d "$DART_BIN" ]]; then
    DART_BIN="$SRC_DIR/out/ci/host_debug_unopt/dart-sdk/bin"
  fi
else
  DART_BIN="$1"
fi

DART="$DART_BIN/dart"

if [[ ! -f "$DART" ]]; then
  echo "'$DART' not found"
  echo ""
  echo "To build the Dart SDK, run:"
  echo "  flutter/tools/gn --unoptimized --runtime-mode=debug"
  echo "  ninja -C out/host_debug_unopt"
  exit 1
fi

echo "Using dart from $DART_BIN"
"$DART" --version
echo ""

# The web SDK does not use pub_get_offline.py, so we need to run pub get first:
echo "Running 'pub get' in the Web SDK"
echo ""
(cd "$FLUTTER_DIR/lib/web_ui"; "$DART" pub --suppress-analytics get)
(cd "$FLUTTER_DIR/web_sdk"; "$DART" pub --suppress-analytics get)
(cd "$FLUTTER_DIR/web_sdk/web_test_utils"; "$DART" pub --suppress-analytics get)
(cd "$FLUTTER_DIR/web_sdk/web_engine_tester"; "$DART" pub --suppress-analytics get)

# Check *all* the Dart code in the engine.
echo "Analyzing the Flutter engine"
echo ""
"$DART" analyze --suppress-analytics --fatal-infos --fatal-warnings "$FLUTTER_DIR"
echo ""

# Check that dart libraries conform.
echo "Checking the integrity of the Web SDK"
WEB_SDK_TEST_FILES="$FLUTTER_DIR/web_sdk/test/*"
for testFile in $WEB_SDK_TEST_FILES
do
  echo "Running $testFile"
  (cd "$FLUTTER_DIR"; FLUTTER_DIR="$FLUTTER_DIR" "$DART" --enable-asserts $testFile)
done
