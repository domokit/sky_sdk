#!/bin/bash
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#
### Checks out the version of Flutter engine in your Fuchsia source tree.
### This is necessary to avoid skew between the version of the Dart VM used in
### the flutter_runner and the version of the Dart SDK and VM used by the
### Flutter toolchain. See
### https://github.com/flutter/flutter/wiki/Compiling-the-engine#important-dart-version-synchronization-on-fuchsia
### for more details.
###
### Example:
###   $ ./checkout_fuchsia_revision.sh

set -e  # Fail on any error.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"/lib/vars.sh || exit $?

ensure_fuchsia_dir
ensure_engine_dir

engine-info "Fetching upstream/main to make sure we recognize Fuchsia's version of Flutter Engine..."
git -C "$ENGINE_DIR"/flutter fetch upstream

fuchsia_flutter_git_revision="$(cat $FUCHSIA_DIR/integration/jiri.lock | grep -A 1 "\"package\": \"flutter/fuchsia\"" | grep "git_revision" | tr ":" "\n" | sed -n 3p | tr "\"" "\n" | sed -n 1p)"
engine-info "Checking out Fuchsia's revision of Flutter Engine ($fuchsia_flutter_git_revision)..."
git -C "$ENGINE_DIR"/flutter checkout $fuchsia_flutter_git_revision

engine-info "Syncing the Flutter Engine dependencies..."
pushd $ENGINE_DIR
gclient sync -D
popd

echo "Done. You're now working on Fuchsia's version of Flutter Engine ($fuchsia_flutter_git_revision)."
