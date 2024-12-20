#!/bin/bash
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# This script is currently used only by automation to collect and upload
# metrics and expects $ENGINE_PATH to be set.

set -ex

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
  while [[ -h "$file" ]]; do
    cd -P "$(dirname -- "$file")"
    file="$(readlink -- "$file")"
    cd -P "$(dirname -- "$file")"
    file="$PWD/$(basename -- "$file")"
  done
  echo "$file"
)

function dart_bin() {
  dart_path="$1/flutter/third_party/dart/tools/sdks/dart-sdk/bin"
  if [[ ! -e "$dart_path" ]]; then
    dart_path="$1/third_party/dart/tools/sdks/dart-sdk/bin"
  fi
  echo "$dart_path"
}

SCRIPT_DIR=$(follow_links "$(dirname -- "${BASH_SOURCE[0]}")")
SRC_DIR="$(cd "$SCRIPT_DIR/../../.."; pwd -P)"
DART_BIN=$(dart_bin "$SRC_DIR")
DART="${DART_BIN}/dart"

VARIANT=$1
shift 1

cd "$SCRIPT_DIR"
"$DART" bin/parse_and_send.dart \
  --json $ENGINE_PATH/src/out/${VARIANT}/txt_benchmarks.json "$@"
"$DART" bin/parse_and_send.dart \
  --json $ENGINE_PATH/src/out/${VARIANT}/fml_benchmarks.json "$@"
"$DART" bin/parse_and_send.dart \
  --json $ENGINE_PATH/src/out/${VARIANT}/shell_benchmarks.json "$@"
"$DART" bin/parse_and_send.dart \
  --json $ENGINE_PATH/src/out/${VARIANT}/ui_benchmarks.json "$@"
"$DART" bin/parse_and_send.dart \
  --json $ENGINE_PATH/src/out/${VARIANT}/display_list_builder_benchmarks.json "$@"
"$DART" bin/parse_and_send.dart \
  --json $ENGINE_PATH/src/out/${VARIANT}/display_list_region_benchmarks.json "$@"
"$DART" bin/parse_and_send.dart \
  --json $ENGINE_PATH/src/out/${VARIANT}/display_list_transform_benchmarks.json "$@"
"$DART" bin/parse_and_send.dart \
  --json $ENGINE_PATH/src/out/${VARIANT}/geometry_benchmarks.json "$@"
