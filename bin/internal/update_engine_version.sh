#!/usr/bin/env bash
# Copyright 2014 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# Want to test this script?
# $ cd dev/tools
# $ dart test test/update_engine_version_test.dart

# ---------------------------------- NOTE ---------------------------------- #
#
# Please keep the logic in this file consistent with the logic in the
# `update_engine_version.ps1` script in the same directory to ensure that Flutter
# continues to work across all platforms!
#
# -------------------------------------------------------------------------- #

set -e

# Allow overriding the intended engine version via FLUTTER_PREBUILT_ENGINE_VERSION.
#
# This is for systems, such as Github Actions, where we know ahead of time the
# base-ref we want to use (to download the engine binaries and avoid trying
# to compute one below), or for the Dart HH bot, which wants to try the current
# Flutter framework/engine with a different Dart SDK.
#
# This environment variable is EXPERIMENTAL. If you are not on the Flutter infra
# or Dart infra teams, this code path might be removed at anytime and cease
# functioning. Please file an issue if you have workflow needs.
if [ -n "${FLUTTER_PREBUILT_ENGINE_VERSION}" ]; then
  ENGINE_VERSION="${FLUTTER_PREBUILT_ENGINE_VERSION}"
fi

FLUTTER_ROOT="$(dirname "$(dirname "$(dirname "${BASH_SOURCE[0]}")")")"

# Test for fusion repository and no environment variable override.
if [ -z "$ENGINE_VERSION" ] && [ -f "$FLUTTER_ROOT/DEPS" ] && [ -f "$FLUTTER_ROOT/engine/src/.gn" ]; then
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
fi

if [[ "$BRANCH" != "stable" && "$BRANCH" != "beta" ]]; then
  # Write the engine version out so downstream tools know what to look for.
  echo $ENGINE_VERSION > "$FLUTTER_ROOT/bin/internal/engine.version"

  # The realm on CI is passed in.
  if [ -n "${FLUTTER_REALM}" ]; then
    echo $FLUTTER_REALM > "$FLUTTER_ROOT/bin/internal/engine.realm"
  fi
fi
