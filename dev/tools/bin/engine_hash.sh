#!/usr/bin/env bash
# Copyright 2014 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# ---------------------------------- NOTE ---------------------------------- #
#
# This file will appear unused within the monorepo. It is used internally
# (in google3) as part of the roll process, and care should be put before
# making changes.
#
# See cl/688973229.
#
# -------------------------------------------------------------------------- #

# Needed because if it is set, cd may print the path it changed to.
unset CDPATH

# On Mac OS, readlink -f doesn't work, so follow_links traverses the path one
# link at a time, and then cds into the link destination and find out where it
# ends up.
#
# The returned filesystem path must be a format usable by Dart's URI parser,
# since the Dart command line tool treats its argument as a file URI, not a
# filename. For instance, multiple consecutive slashes should be reduced to a
# single slash, since double-slashes indicate a URI "authority", and these are
# supposed to be filenames. There is an edge case where this will return
# multiple slashes: when the input resolves to the root directory. However, if
# that were the case, we wouldn't be running this shell, so we don't do anything
# about it.
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

PROG_NAME="$(follow_links "${BASH_SOURCE[0]}")"
BIN_DIR="$(cd "${PROG_NAME%/*}" ; pwd -P)"
FLUTTER_ROOT="$(cd "${BIN_DIR}/../../.." ; pwd -P)"

# Allow using a mock git for testing.
if [ -z "$GIT" ]; then
    # By default, use git on PATH.
    GIT_BIN="git"
else
    # Use the provide GIT executable.
    GIT_BIN="$GIT"
fi

# Test for fusion repository
if [ -f "$FLUTTER_ROOT/DEPS" ]; then
    ENGINE_VERSION=$($GIT_BIN -C "$FLUTTER_ROOT" merge-base HEAD origin/master)
elif [ -f "$FLUTTER_ROOT/bin/internal/engine.version" ]; then
    ENGINE_VERSION=$(cat "$FLUTTER_ROOT/bin/internal/engine.version")
else
    >&2 echo "Not a valid FLUTTER_ROOT: $FLUTTER_ROOT"
    exit 1
fi

echo $ENGINE_VERSION
