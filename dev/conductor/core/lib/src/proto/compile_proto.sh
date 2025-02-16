#!/usr/bin/env bash
# Copyright 2014 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

set -euo pipefail

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
DIR="$(cd "${PROG_NAME%/*}" ; pwd -P)"

# Ensure dart-sdk is cached
"$DIR/../../../../../../bin/dart" --version

if ! type protoc >/dev/null 2>&1; then
  PROTOC_LINK='https://grpc.io/docs/protoc-installation/'
  echo "Error! \"protoc\" binary required on path."
  echo "See $PROTOC_LINK for more information."
  exit 1
fi

if ! type dart >/dev/null 2>&1; then
  echo "Error! \"dart\" binary required on path."
  exit 1
fi

# Use null-safe protoc_plugin
dart pub global activate protoc_plugin 21.1.2

protoc --dart_out="$DIR" --proto_path="$DIR" "$DIR/conductor_state.proto"

for SOURCE_FILE in $(ls "$DIR"/*.pb*.dart); do
  # Format in place file
  dart format --output=write "$SOURCE_FILE"

  # Create temp copy with the license header prepended
  cp "$DIR/license_header.txt" "${SOURCE_FILE}.tmp"

  # Add an extra newline required by analysis (analysis also prevents
  # license_header.txt from having the trailing newline)
  echo '' >> "${SOURCE_FILE}.tmp"

  cat "$SOURCE_FILE" >> "${SOURCE_FILE}.tmp"

  # Move temp version (with license) over the original
  mv "${SOURCE_FILE}.tmp" "$SOURCE_FILE"
done
