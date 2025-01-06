#!/usr/bin/env bash
# Copyright 2014 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.


# ---------------------------------- NOTE ---------------------------------- #
#
# Please keep the logic in this file consistent with the logic in the
# `update_engine_version.ps1` script in the same directory to ensure that Flutter
# continues to work across all platforms!
#
# -------------------------------------------------------------------------- #

set -e

FLUTTER_ROOT="$(dirname "$(dirname "$(dirname "${BASH_SOURCE[0]}")")")"

# Test for fusion repository
if [ -f "$FLUTTER_ROOT/DEPS" ] && [ -f "$FLUTTER_ROOT/engine/src/.gn" ]; then
  BRANCH=$(git -C "$FLUTTER_ROOT" rev-parse --abbrev-ref HEAD)
  # In a fusion repository; the engine.version comes from the git hashes.
  if [ -z "${LUCI_CONTEXT}" ]; then
    set +e
    # Run the git command and capture the exit code
    git -C "$FLUTTER_ROOT" remote get-url upstream > /dev/null 2>&1
    exit_code=$?
    set -e

    if [[ $exit_code -eq 0 ]]; then
      ENGINE_VERSION=$(git -C "$FLUTTER_ROOT" merge-base HEAD upstream/master)
    else
      ENGINE_VERSION=$(git -C "$FLUTTER_ROOT" merge-base HEAD origin/master)
    fi
  else
    ENGINE_VERSION=$(git -C "$FLUTTER_ROOT" rev-parse HEAD)
  fi

  if [[ "$BRANCH" != "stable" && "$BRANCH" != "beta" ]]; then
    # Write the engine version out so downstream tools know what to look for.
    echo $ENGINE_VERSION > "$FLUTTER_ROOT/bin/internal/engine.version"

    # The realm on CI is passed in.
    if [ -n "${FLUTTER_REALM}" ]; then
      echo $FLUTTER_REALM > "$FLUTTER_ROOT/bin/internal/engine.realm"
    fi
  fi
fi
