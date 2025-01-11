#!/bin/bash

# TODO(matanlurey): Remove all references are gone and using run_ios_tests.dart.
# See https://github.com/flutter/flutter/issues/143953 for tracking.

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

"$DART" \
  --disable-dart-dev \
  testing/ios_scenario_app/bin/run_ios_tests.dart \
  "$@"
