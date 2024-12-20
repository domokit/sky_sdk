#!/usr/bin/env bash
# Copyright 2014 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.


# ---------------------------------- NOTE ---------------------------------- #
#
# Please keep the logic in this file consistent with the logic in the
# `update_dart_sdk.ps1` script in the same directory to ensure that Flutter
# continues to work across all platforms!
#
# -------------------------------------------------------------------------- #

set -e

FLUTTER_ROOT="$(dirname "$(dirname "$(dirname "${BASH_SOURCE[0]}")")")"

DART_SDK_PATH="$FLUTTER_ROOT/bin/cache/dart-sdk"
DART_SDK_PATH_OLD="$DART_SDK_PATH.old"
ENGINE_STAMP="$FLUTTER_ROOT/bin/cache/engine-dart-sdk.stamp"
ENGINE_REALM=""
OS="$(uname -s)"

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
ENGINE_VERSION=$(cat "$FLUTTER_ROOT/bin/internal/engine.version")
ENGINE_REALM=$(cat "$FLUTTER_ROOT/bin/internal/engine.realm" | tr -d '[:space:]')

if [ ! -f "$ENGINE_STAMP" ] || [ "$ENGINE_VERSION" != `cat "$ENGINE_STAMP"` ]; then
  command -v curl > /dev/null 2>&1 || {
    >&2 echo
    >&2 echo 'Missing "curl" tool. Unable to download Dart SDK.'
    case "$OS" in
      Darwin)
        >&2 echo 'Consider running "brew install curl".'
        ;;
      Linux)
        >&2 echo 'Consider running "sudo apt-get install curl".'
        ;;
      *)
        >&2 echo "Please install curl."
        ;;
    esac
    echo
    exit 1
  }
  command -v unzip > /dev/null 2>&1 || {
    >&2 echo
    >&2 echo 'Missing "unzip" tool. Unable to extract Dart SDK.'
    case "$OS" in
      Darwin)
        echo 'Consider running "brew install unzip".'
        ;;
      Linux)
        echo 'Consider running "sudo apt-get install unzip".'
        ;;
      *)
        echo "Please install unzip."
        ;;
    esac
    echo
    exit 1
  }

  # `uname -m` may be running in Rosetta mode, instead query sysctl
  if [ "$OS" = 'Darwin' ]; then
    # Allow non-zero exit so we can do control flow
    set +e
    # -n means only print value, not key
    QUERY="sysctl -n hw.optional.arm64"
    # Do not wrap $QUERY in double quotes, otherwise the args will be treated as
    # part of the command
    QUERY_RESULT=$($QUERY 2>/dev/null)
    if [ $? -eq 1 ]; then
      # If this command fails, we're certainly not on ARM
      ARCH='x64'
    elif [ "$QUERY_RESULT" = '0' ]; then
      # If this returns 0, we are also not on ARM
      ARCH='x64'
    elif [ "$QUERY_RESULT" = '1' ]; then
      ARCH='arm64'
    else
      >&2 echo "'$QUERY' returned unexpected output: '$QUERY_RESULT'"
      exit 1
    fi
    set -e
  else
    # On x64 stdout is "uname -m: x86_64"
    # On arm64 stdout is "uname -m: aarch64, arm64_v8a"
    case "$(uname -m)" in
      x86_64)
        ARCH="x64"
        ;;
      *)
        ARCH="arm64"
        ;;
    esac
  fi

  case "$OS" in
    Darwin)
      DART_ZIP_NAME="dart-sdk-darwin-${ARCH}.zip"
      IS_USER_EXECUTABLE="-perm +100"
      ;;
    Linux)
      DART_ZIP_NAME="dart-sdk-linux-${ARCH}.zip"
      IS_USER_EXECUTABLE="-perm /u+x"
      ;;
    MINGW* | MSYS* )
      DART_ZIP_NAME="dart-sdk-windows-x64.zip"
      IS_USER_EXECUTABLE="-perm /u+x"
      ;;
    *)
      echo "Unknown operating system. Cannot install Dart SDK."
      exit 1
      ;;
  esac

  >&2 echo "Downloading $OS $ARCH Dart SDK from Flutter engine $ENGINE_VERSION..."

  # Use the default find if possible.
  if [ -e /usr/bin/find ]; then
    FIND=/usr/bin/find
  else
    FIND=find
  fi

  DART_SDK_BASE_URL="${FLUTTER_STORAGE_BASE_URL:-https://storage.googleapis.com}${ENGINE_REALM:+/$ENGINE_REALM}"
  DART_SDK_URL="$DART_SDK_BASE_URL/flutter_infra_release/flutter/$ENGINE_VERSION/$DART_ZIP_NAME"

  # if the sdk path exists, copy it to a temporary location
  if [ -d "$DART_SDK_PATH" ]; then
    rm -rf "$DART_SDK_PATH_OLD"
    mv "$DART_SDK_PATH" "$DART_SDK_PATH_OLD"
  fi

  # install the new sdk
  rm -rf -- "$DART_SDK_PATH"
  mkdir -m 755 -p -- "$DART_SDK_PATH"
  DART_SDK_ZIP="$FLUTTER_ROOT/bin/cache/$DART_ZIP_NAME"

  # Conditionally set verbose flag for LUCI
  verbose_curl=""
  if [[ -n "$LUCI_CI" ]]; then
    verbose_curl="--verbose"
  fi

  curl ${verbose_curl} --retry 3 --continue-at - --location --output "$DART_SDK_ZIP" "$DART_SDK_URL" 2>&1 || {
    curlExitCode=$?
    # Handle range errors specially: retry again with disabled ranges (`--continue-at -` argument)
    # When this could happen:
    # - missing support of ranges in proxy servers
    # - curl with broken handling of completed downloads
    #   This is not a proper fix, but doesn't require any user input
    # - mirror of flutter storage without support of ranges
    #
    # 33  HTTP range error. The range "command" didn't work.
    # https://man7.org/linux/man-pages/man1/curl.1.html#EXIT_CODES
    if [ $curlExitCode != 33 ]; then
      return $curlExitCode
    fi
    curl ${verbose_curl} --retry 3 --location --output "$DART_SDK_ZIP" "$DART_SDK_URL" 2>&1
  } || {
    >&2 echo
    >&2 echo "Failed to retrieve the Dart SDK from: $DART_SDK_URL"
    >&2 echo "If you're located in China, please see this page:"
    >&2 echo "  https://flutter.dev/community/china"
    >&2 echo
    rm -f -- "$DART_SDK_ZIP"
    exit 1
  }
  unzip -o -q "$DART_SDK_ZIP" -d "$FLUTTER_ROOT/bin/cache" || {
    >&2 echo
    >&2 echo "It appears that the downloaded file is corrupt; please try again."
    >&2 echo "If this problem persists, please report the problem at:"
    >&2 echo "  https://github.com/flutter/flutter/issues/new?template=1_activation.yml"
    >&2 echo
    rm -f -- "$DART_SDK_ZIP"
    exit 1
  }
  rm -f -- "$DART_SDK_ZIP"
  $FIND "$DART_SDK_PATH" -type d -exec chmod 755 {} \;
  $FIND "$DART_SDK_PATH" -type f $IS_USER_EXECUTABLE -exec chmod a+x,a+r {} \;
  echo "$ENGINE_VERSION" > "$ENGINE_STAMP"

  # delete any temporary sdk path
  if [ -d "$DART_SDK_PATH_OLD" ]; then
    rm -rf "$DART_SDK_PATH_OLD"
  fi
fi
