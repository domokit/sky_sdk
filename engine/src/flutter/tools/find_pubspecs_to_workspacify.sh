#!/bin/bash

set -e

# This script finds `pubspec.yaml` files that do not contain a top-level
# `resolution: workspace` key-value pair, in order to help identify packages
# as part of <https://github.com/flutter/flutter/issues/147883>.
#
# Usage:
#   tools/find_pubspecs_to_workspacify.sh

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

SCRIPT_DIR=$(follow_links "$(dirname -- "${BASH_SOURCE[0]}")")
SRC_DIR="$(
  cd "$SCRIPT_DIR/../.."
  pwd -P
)"
FLUTTER_DIR="$(
  cd "$SRC_DIR/flutter"
  pwd -P
)"

# Patterns, that, if matched, should be ignored.
IGNORE_PATTERNS=(
  # Root pubspec.yaml file.
  '.*flutter\/pubspec.yaml'

  # Anything fuchsia-related.
  '.*shell\/platform\/fuchsia.*'

  # Anything web-related.
  '.*web_sdk.*'

  # Anything in the `prebuilts` directory.
  '.*prebuilts.*'

  # Anything in the `third_party` directory.
  '.*third_party.*'
)

# Find all pubspec.yaml files that do not contain a top-level `resolution: workspace` key-value pair.
find "$FLUTTER_DIR" -name pubspec.yaml -print0 | while IFS= read -r -d '' pubspec; do
  # Check if the pubspec.yaml file should be ignored.
  for pattern in "${IGNORE_PATTERNS[@]}"; do
    if [[ "$pubspec" =~ $pattern ]]; then
      continue 2
    fi
  done

  # Check if the pubspec.yaml file contains a top-level `resolution: workspace` key-value pair.
  if ! grep -q '^resolution: workspace' "$pubspec"; then
    echo "$pubspec"
  fi
done
