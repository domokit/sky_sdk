#!/bin/bash
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#
### Tests build_and_copy_to_fuchsia.sh by running it with various arguments.
### This script doesn't assert the results but just checks that the script runs
### successfully every time.
###
### This script doesn't run on CQ, it's only used for local testing.

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"/../lib/vars.sh || exit $?

ensure_fuchsia_dir
ensure_engine_dir

set -e  # Fail on any error.

# TODO(akbiggs): Remove prebuilts before each build to check that the right set of
# prebuilts is being deployed. I tried this initially but gave up because
# build_and_copy_to_fuchsia.sh would fail when the prebuilt directories were missing.

engine-info "Testing build_and_copy_to_fuchsia.sh with no args..."
"$ENGINE_DIR"/flutter/tools/fuchsia/devshell/build_and_copy_to_fuchsia.sh

engine-info "Testing build_and_copy_to_fuchsia.sh --unoptimized..."
$ENGINE_DIR/flutter/tools/fuchsia/devshell/build_and_copy_to_fuchsia.sh --unoptimized

engine-info "Testing build_and_copy_to_fuchsia.sh --runtime-mode profile..."
$ENGINE_DIR/flutter/tools/fuchsia/devshell/build_and_copy_to_fuchsia.sh --runtime-mode profile

engine-info "Testing build_and_copy_to_fuchsia.sh --runtime-mode release..."
$ENGINE_DIR/flutter/tools/fuchsia/devshell/build_and_copy_to_fuchsia.sh --runtime-mode release

engine-info "Testing build_and_copy_to_fuchsia.sh --fuchsia-cpu arm64..."
$ENGINE_DIR/flutter/tools/fuchsia/devshell/build_and_copy_to_fuchsia.sh --fuchsia-cpu arm64

engine-info "Testing build_and_copy_to_fuchsia.sh --no-prebuilt-dart-sdk..."
$ENGINE_DIR/flutter/tools/fuchsia/devshell/build_and_copy_to_fuchsia.sh --no-prebuilt-dart-sdk
