#!/usr/bin/env bash
# Copyright 2014 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# This should match the ci.bat file in this directory.

# This is called from the LUCI recipes:
# https://github.com/flutter/flutter/blob/main/dev/bots/suite_runners/run_customer_testing_tests.dart

set -e

function script_location() {
  local script_location="${BASH_SOURCE[0]}"
  # Resolve symlinks
  while [[ -h "$script_location" ]]; do
    DIR="$(cd -P "$( dirname "$script_location")" >/dev/null && pwd)"
    script_location="$(readlink "$script_location")"
    [[ "$script_location" != /* ]] && script_location="$DIR/$script_location"
  done
  cd -P "$(dirname "$script_location")" >/dev/null && pwd
}

# So that users can run this script from anywhere and it will work as expected.
cd "$(script_location)"

# This script does not assume that "flutter update-packages" has been
# run, to allow CIs to save time by skipping that steps since it's
# largely not needed to run the flutter/tests tests.
#
# However, we do need to update this directory.
dart pub get

# Run the cross-platform script.
../../bin/dart run ci.dart
