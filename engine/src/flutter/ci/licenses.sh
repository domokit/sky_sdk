#!/bin/bash
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# Do not exit when a non-zero return value is encountered to output all errors.
# See: https://github.com/flutter/flutter/issues/131680
# set -e
shopt -s nullglob

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
SRC_DIR="$(
  cd "$SCRIPT_DIR/../.."
  pwd -P
)"
DART_BIN=$(dart_bin "$SRC_DIR")
PATH="$DART_BIN:$PATH"

# Use:
#   env VERBOSE=1 ./ci/licenses.sh
# to turn on verbose progress report printing. Set it to 2 to also output
# information about which patterns are taking the most time.
QUIET="--quiet"
if [[ "${VERBOSE}" == "1" ]]; then
  QUIET=""
fi
if [[ "${VERBOSE}" == "2" ]]; then
  QUIET="--verbose"
fi

echo "Verifying license script is still happy..."
echo "Using dart from: $(command -v dart)"

untracked_files="$(
  cd "$SRC_DIR/flutter"
  git status --ignored --short | grep -E "^!" | awk "{print\$2}"
)"
untracked_count="$(echo "$untracked_files" | wc -l)"
if [[ $untracked_count -gt 0 ]]; then
  echo ""
  echo "WARNING: There are $untracked_count untracked/ignored files or directories in the flutter repository."
  echo "False positives may occur."
  echo "You can use 'git clean -dxf' in the flutter dir to clean out these files."
  echo "BUT, be warned that this will recursively remove all these files and directories:"
  echo "$untracked_files"
  echo ""
fi

dart --version

# Runs the tests for the license script.
function run_tests() (
  cd "$SRC_DIR/flutter/tools/licenses"
  find . -name "*_test.dart" | xargs -n 1 dart --enable-asserts
)

# Collects the license information from the repo.
# Runs in a subshell.
function collect_licenses() (
  cd "$SRC_DIR/flutter/tools/licenses"
  # `--interpret_irregexp`` tells dart to use interpreter mode for running
  # the regexp matching, rather than generating machine code for it.
  # For very large RegExps that are currently used in license script using
  # interpreter is faster than using unoptimized machine code, which has
  # no chance of being optimized(due to its size).
  dart \
    --enable-asserts \
    --interpret_irregexp \
    lib/main.dart \
    --src ../../.. \
    --out ../../../out/license_script_output \
    --golden ../../ci/licenses_golden \
    "${QUIET}"
)

# Verifies the licenses in the repo.
# Runs in a subshell.
function verify_licenses() (
  local exitStatus=0
  cd "$SRC_DIR"

  # These files trip up the script on Mac OS X.
  find . -name ".DS_Store" -exec rm -f {} \;

  collect_licenses

  for f in out/license_script_output/licenses_*; do
    if ! cmp -s "flutter/ci/licenses_golden/$(basename "$f")" "$f"; then
      echo "============================= ERROR ============================="
      echo "License script got different results than expected for $f."
      echo "Please rerun the licenses script locally to verify that it is"
      echo "correctly catching any new licenses for anything you may have"
      echo "changed, and then update this file:"
      echo "  flutter/sky/packages/sky_engine/LICENSE"
      echo "For more information, see the script in:"
      echo "  https://github.com/flutter/engine/tree/main/tools/licenses"
      echo ""
      diff -U 6 "flutter/ci/licenses_golden/$(basename "$f")" "$f"
      echo "================================================================="
      echo ""
      exitStatus=1
    fi
  done

  echo "Verifying license tool signature..."
  if ! cmp -s "flutter/ci/licenses_golden/tool_signature" "out/license_script_output/tool_signature"; then
    echo "============================= ERROR ============================="
    echo "The license tool signature has changed. This is expected when"
    echo "there have been changes to the license tool itself. Licenses have"
    echo "been re-computed for all components. If only the license script has"
    echo "changed, no diffs are typically expected in the output of the"
    echo "script. Verify the output, and if it looks correct, update the"
    echo "license tool signature golden file:"
    echo "  ci/licenses_golden/tool_signature"
    echo "For more information, see the script in:"
    echo "  https://github.com/flutter/engine/tree/main/tools/licenses"
    echo ""
    diff -U 6 "flutter/ci/licenses_golden/tool_signature" "out/license_script_output/tool_signature"
    echo "================================================================="
    echo ""
    exitStatus=1
  fi

  echo "Verifying excluded files list..."
  if ! cmp -s "flutter/ci/licenses_golden/excluded_files" "out/license_script_output/excluded_files"; then
    echo "============================= ERROR ============================="
    echo "The license is excluding a different number of files than previously."
    echo "This is only expected when new non-source files have been introduced."
    echo "Verify that all the newly ignored files are definitely not shipped with"
    echo "any binaries that we compile (including impellerc and Wasm)."
    echo "If the changes look correct, update this file:"
    echo "  ci/licenses_golden/excluded_files"
    echo "For more information, see the script in:"
    echo "  https://github.com/flutter/engine/tree/main/tools/licenses"
    echo ""
    diff -U 6 "flutter/ci/licenses_golden/excluded_files" "out/license_script_output/excluded_files"
    echo "================================================================="
    echo ""
    exitStatus=1
  fi

  echo "Checking license count in licenses_flutter..."

  local actualLicenseCount
  actualLicenseCount="$(tail -n 1 flutter/ci/licenses_golden/licenses_flutter | tr -dc '0-9')"
  local expectedLicenseCount=917

  if [[ $actualLicenseCount -ne $expectedLicenseCount ]]; then
    echo "=============================== ERROR ==============================="
    echo "The total license count in flutter/ci/licenses_golden/licenses_flutter"
    echo "changed from $expectedLicenseCount to $actualLicenseCount."
    echo "It's very likely that this is an unintentional change. Please"
    echo "double-check that all newly added files have a BSD-style license"
    echo "header with the following copyright:"
    echo "    Copyright 2013 The Flutter Authors. All rights reserved."
    echo "Files in 'third_party/txt' may have an Apache license header instead."
    echo "If you're absolutely sure that the change in license count is"
    echo "intentional, update 'flutter/ci/licenses.sh' with the new count."
    echo "================================================================="
    echo ""
    exitStatus=1
  fi

  if [[ $exitStatus -eq 0 ]]; then
    echo "Licenses are as expected."
  fi
  return $exitStatus
)

run_tests
verify_licenses
