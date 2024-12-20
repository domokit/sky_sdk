#!/bin/bash
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#
### Tests run_unit_tests.sh by running it with various arguments.
### This script doesn't assert the results but just checks that the script runs
### successfully every time.
###
### This script doesn't run on CQ, it's only used for local testing.

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"/../lib/vars.sh || exit $?

ensure_engine_dir

set -e  # Fail on any error.

engine-info "Testing run_unit_tests.sh --package-filter flutter_runner_tests-0.far..."
"$ENGINE_DIR"/flutter/tools/fuchsia/devshell/run_unit_tests.sh --package-filter "flutter_runner_tests-0.far"

engine-info "Testing run_unit_tests.sh --package-filter flutter_runner_tests-0.far --count 2..."
"$ENGINE_DIR"/flutter/tools/fuchsia/devshell/run_unit_tests.sh --package-filter "flutter_runner_tests-0.far" --count 2


engine-info "Testing run_unit_tests.sh --package-filter flutter_runner_tests-0.far --gtest-filter *FlatlandConnection*..."
"$ENGINE_DIR"/flutter/tools/fuchsia/devshell/run_unit_tests.sh --package-filter "flutter_runner_tests-0.far" --gtest-filter "*FlatlandConnection*"

engine-info "Testing run_unit_tests.sh with no args..."
"$ENGINE_DIR"/flutter/tools/fuchsia/devshell/run_unit_tests.sh

