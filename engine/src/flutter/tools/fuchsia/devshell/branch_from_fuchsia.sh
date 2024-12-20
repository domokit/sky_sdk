#!/bin/bash
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#
### Starts a new branch from Fuchsia's checkout of the Flutter engine.
### This is necessary to avoid skew between the version of the Dart VM used in
### the flutter_runner and the version of the Dart SDK and VM used by the
### Flutter toolchain. See
### https://github.com/flutter/flutter/wiki/Compiling-the-engine#important-dart-version-synchronization-on-fuchsia
### for more details.
###
### Example:
###   $ ./branch_from_fuchsia.sh my_new_feature_branch

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"/lib/vars.sh || exit $?

engine-info "git checkout Fuchsia's version of Flutter Engine..."
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"/checkout_fuchsia_revision.sh || exit $?

engine-info "Creating new branch '$1'."
git checkout -b $1
if [ $? -ne 0 ]
then
    engine-error "Failed to create new branch '$1'. Restoring previous checkout."
    git checkout -
    exit $?
fi
