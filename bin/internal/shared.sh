#!/usr/bin/env bash
# Copyright 2014 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# ---------------------------------- NOTE ---------------------------------- #
#
# Please keep the logic in this file consistent with the logic in the
# `shared.bat` script in the same directory to ensure that Flutter & Dart continue
# to work across all platforms!
#
# -------------------------------------------------------------------------- #

set -e

# Needed because if it is set, cd may print the path it changed to.
unset CDPATH

function pub_upgrade_with_retry {
  local total_tries="10"
  local remaining_tries=$((total_tries - 1))
  while [[ "$remaining_tries" -gt 0 ]]; do
    (cd "$FLUTTER_TOOLS_DIR" && "$DART" pub upgrade --suppress-analytics) && break
    >&2 echo "Error: Unable to 'pub upgrade' flutter tool. Retrying in five seconds... ($remaining_tries tries left)"
    remaining_tries=$((remaining_tries - 1))
    sleep 5
  done

  if [[ "$remaining_tries" == 0 ]]; then
    >&2 echo "Command 'pub upgrade' still failed after $total_tries tries, giving up."
    return 1
  fi
  return 0
}

# Trap function for removing any remaining lock file at exit.
function _rmlock () {
  [ -n "$FLUTTER_UPGRADE_LOCK" ] && rm -rf "$FLUTTER_UPGRADE_LOCK"
}

# Determines which lock method to use, based on what is available on the system.
# Returns a non-zero value if the lock was not acquired, zero if acquired.
function _lock () {
  if hash flock 2>/dev/null; then
    flock --nonblock --exclusive 7 2>/dev/null
  elif hash shlock 2>/dev/null; then
    shlock -f "$1" -p $$
  else
    mkdir "$1" 2>/dev/null
  fi
}

# Waits for an update lock to be acquired.
#
# To ensure that we don't simultaneously update Dart in multiple parallel
# instances, we try to obtain an exclusive lock on this file descriptor (and
# thus this script's source file) while we are updating Dart and compiling the
# script. To do this, we try to use the command line program "flock", which is
# available on many Unix-like platforms, in particular on most Linux
# distributions. You give it a file descriptor, and it locks the corresponding
# file, having inherited the file descriptor from the shell.
#
# Complicating matters, there are two major scenarios where this will not
# work.
#
# The first is if the platform doesn't have "flock", for example on macOS. There
# is not a direct equivalent, so on platforms that don't have flock, we fall
# back to using trying to use the shlock command, and if that doesn't exist,
# then we use mkdir as an atomic operation to create a lock directory. If mkdir
# is able to create the directory, then the lock is acquired. To determine if we
# have "flock" or "shlock" available, we use the "hash" shell built-in.
#
# The second complication is on network file shares. On NFS, to obtain an
# exclusive lock you need a file descriptor that is open for writing. Thus, we
# ignore errors from flock by redirecting all output to /dev/null, since users
# will typically not care about errors from flock and are more likely to be
# confused by them than helped. The "shlock" method doesn't work for network
# shares, since it is PID-based. The "mkdir" method does work over NFS
# implementations that support atomic directory creation (which is most of
# them). The "schlock" and "flock" commands are more reliable than the mkdir
# method, however, or we would use mkdir in all cases.
#
# The upgrade_flutter function calling _wait_for_lock is executed in a subshell
# with a redirect that pipes the source of this script into file descriptor 7.
# A flock lock is released when this subshell exits and file descriptor 7 is
# closed. The mkdir lock is released via an exit trap from the subshell that
# deletes the lock directory.
function _wait_for_lock () {
  FLUTTER_UPGRADE_LOCK="$FLUTTER_ROOT/bin/cache/.upgrade_lock"
  local waiting_message_displayed
  while ! _lock "$FLUTTER_UPGRADE_LOCK"; do
    if [[ -z $waiting_message_displayed ]]; then
      # Print with a return so that if the Dart code also prints this message
      # when it does its own lock, the message won't appear twice. Be sure that
      # the clearing printf below has the same number of space characters.
      printf "Waiting for another flutter command to release the startup lock...\r" >&2;
      waiting_message_displayed="true"
    fi
    sleep .1;
  done
  if [[ $waiting_message_displayed == "true" ]]; then
    # Clear the waiting message so it doesn't overlap any following text.
    printf "                                                                  \r" >&2;
  fi
  unset waiting_message_displayed
  # If the lock file is acquired, make sure that it is removed on exit.
  trap _rmlock INT TERM EXIT
}

# This function is always run in a subshell. Running the function in a subshell
# is required to make sure any lock directory is cleaned up by the exit trap in
# _wait_for_lock.
function upgrade_flutter () (
  mkdir -p "$FLUTTER_ROOT/bin/cache"

  # Ensure the engine.version is populated
  "$FLUTTER_ROOT/bin/internal/update_engine_version.sh"

  local revision="$(cd "$FLUTTER_ROOT"; git rev-parse HEAD)"
  local compilekey="$revision:$FLUTTER_TOOL_ARGS"

  # Invalidate cache if:
  #  * SNAPSHOT_PATH is not a file, or
  #  * STAMP_PATH is not a file, or
  #  * STAMP_PATH is an empty file, or
  #  * Contents of STAMP_PATH is not what we are going to compile, or
  #  * pubspec.yaml last modified after pubspec.lock
  if [[ ! -f "$SNAPSHOT_PATH" || \
        ! -s "$STAMP_PATH" || \
        "$(cat "$STAMP_PATH")" != "$compilekey" || \
        "$FLUTTER_TOOLS_DIR/pubspec.yaml" -nt "$FLUTTER_TOOLS_DIR/pubspec.lock" ]]; then
    # Waits for the update lock to be acquired. Placing this check inside the
    # conditional allows the majority of flutter/dart installations to bypass
    # the lock entirely, but as a result this required a second verification that
    # the SDK is up to date.
    _wait_for_lock

    # A different shell process might have updated the tool/SDK.
    if [[ -f "$SNAPSHOT_PATH" && -s "$STAMP_PATH" && "$(cat "$STAMP_PATH")" == "$compilekey" && "$FLUTTER_TOOLS_DIR/pubspec.yaml" -ot "$FLUTTER_TOOLS_DIR/pubspec.lock" ]]; then
      exit $?
    fi

    # Fetch Dart...
    rm -f "$FLUTTER_ROOT/version"
    rm -f "$FLUTTER_ROOT/bin/cache/flutter.version.json"
    touch "$FLUTTER_ROOT/bin/cache/.dartignore"
    "$FLUTTER_ROOT/bin/internal/update_dart_sdk.sh"

    if [[ "$BIN_NAME" == 'dart' || "$BIN_NAME" == 'flutter-dev' ]]; then
      # Don't try to build tool
      return
    fi

    >&2 echo Building flutter tool...

    # Prepare packages...
    if [[ "$CI" == "true" || "$BOT" == "true" || "$CONTINUOUS_INTEGRATION" == "true" || "$CHROME_HEADLESS" == "1" ]]; then
      PUB_ENVIRONMENT="$PUB_ENVIRONMENT:flutter_bot"
    else
      export PUB_SUMMARY_ONLY=1
    fi
    export PUB_ENVIRONMENT="$PUB_ENVIRONMENT:flutter_install"
    pub_upgrade_with_retry

    # Move the old snapshot - we can't just overwrite it as the VM might currently have it
    # memory mapped (e.g. on flutter upgrade). For downloading a new dart sdk the folder is moved,
    # so we take the same approach of moving the file here.
    SNAPSHOT_PATH_OLD="$SNAPSHOT_PATH.old"
    if [ -f "$SNAPSHOT_PATH" ]; then
      mv "$SNAPSHOT_PATH" "$SNAPSHOT_PATH_OLD"
    fi

    # Compile...
    "$DART" --verbosity=error $FLUTTER_TOOL_ARGS --snapshot="$SNAPSHOT_PATH" --snapshot-kind="app-jit" --packages="$FLUTTER_TOOLS_DIR/.dart_tool/package_config.json" --no-enable-mirrors "$SCRIPT_PATH" > /dev/null
    echo "$compilekey" > "$STAMP_PATH"

    # Delete any temporary snapshot path.
    if [ -f "$SNAPSHOT_PATH_OLD" ]; then
      rm -f "$SNAPSHOT_PATH_OLD"
    fi
  fi
  # The exit here is extraneous since the function is run in a subshell, but
  # this serves as documentation that running the function in a subshell is
  # required to make sure any lock directory created by mkdir is cleaned up.
  exit $?
)

# This function is intended to be executed by entrypoints (e.g. `//bin/flutter`
# and `//bin/dart`). PROG_NAME and BIN_DIR should already be set by those
# entrypoints.
function shared::execute() {
  export FLUTTER_ROOT="$(cd "${BIN_DIR}/.." ; pwd -P)"

  # If present, run the bootstrap script first
  BOOTSTRAP_PATH="$FLUTTER_ROOT/bin/internal/bootstrap.sh"
  if [ -f "$BOOTSTRAP_PATH" ]; then
    source "$BOOTSTRAP_PATH"
  fi

  FLUTTER_TOOLS_DIR="$FLUTTER_ROOT/packages/flutter_tools"
  SNAPSHOT_PATH="$FLUTTER_ROOT/bin/cache/flutter_tools.snapshot"
  STAMP_PATH="$FLUTTER_ROOT/bin/cache/flutter_tools.stamp"
  SCRIPT_PATH="$FLUTTER_TOOLS_DIR/bin/flutter_tools.dart"
  DART_SDK_PATH="$FLUTTER_ROOT/bin/cache/dart-sdk"

  DART="$DART_SDK_PATH/bin/dart"

  # If running over git-bash, overrides the default UNIX executables with win32
  # executables
  case "$(uname -s)" in
    MINGW* | MSYS* )
      DART="$DART.exe"
      ;;
  esac

  # Test if running as superuser – but don't warn if running within Docker or CI.
  if [[ "$EUID" == "0" && ! -f /.dockerenv && "$CI" != "true" && "$BOT" != "true" && "$CONTINUOUS_INTEGRATION" != "true" ]]; then
    >&2 echo "   Woah! You appear to be trying to run flutter as root."
    >&2 echo "   We strongly recommend running the flutter tool without superuser privileges."
    >&2 echo "  /"
    >&2 echo "📎"
  fi

  # Test if Git is available on the Host
  if ! hash git 2>/dev/null; then
    >&2 echo "Error: Unable to find git in your PATH."
    exit 1
  fi
  # Test if the flutter directory is a git clone (otherwise git rev-parse HEAD
  # would fail)
  # This test is omitted, if flutter has been provided as packaged
  # software within Linux distribution repositories. In this case,
  # evidence of the git history is purged from the package, thus this
  # check would fail.
  current_dir=$(pwd)
  if [ ! -e "$FLUTTER_ROOT/.git" ] && [ "$FLUTTER_ROOT" != "/usr/local/bin/flutter" ]; then
    >&2 echo "Error: The Flutter directory is not a clone of the GitHub project."
    >&2 echo "       The flutter tool requires Git in order to operate properly;"
    >&2 echo "       to install Flutter, see the instructions at:"
    >&2 echo "       https://flutter.dev/get-started"
    exit 1
  fi

  BIN_NAME="$(basename "$PROG_NAME")"

  # File descriptor 7 is prepared here so that we can use it with
  # flock(1) in _lock() (see above).
  #
  # We use number 7 because it's a luckier number than 3; luck is
  # important when making locks work reliably. Also because that way
  # if anyone is redirecting other file descriptors there's less
  # chance of a conflict.
  #
  # In any case, the file we redirect into this file descriptor is
  # this very source file you are reading right now, because that's
  # the only file we can truly guarantee exists, since we're running
  # it. We don't use PROG_NAME because otherwise if you run `dart` and
  # `flutter` simultaneously they'll end up using different lock files
  # and will corrupt each others' downloads.
  #
  # SHARED_NAME itself is prepared by the caller script.
  #
  if [ "$FLUTTER_ROOT" != "/usr/local/bin/flutter" ]; then
    upgrade_flutter 7< "$SHARED_NAME"
  fi
  case "$BIN_NAME" in
    flutter-dev)
      # FLUTTER_TOOL_ARGS aren't quoted below, because it is meant to be
      # considered as separate space-separated args.
      exec "$DART" run --resident --packages="$FLUTTER_TOOLS_DIR/.dart_tool/package_config.json" $FLUTTER_TOOL_ARGS "$SCRIPT_PATH" "$@"
      ;;
    flutter*)
      # FLUTTER_TOOL_ARGS aren't quoted below, because it is meant to be
      # considered as separate space-separated args.
      if [ "$FLUTTER_ROOT" != "/usr/local/bin/flutter" ]; then
        # This condition checks, whether flutter-sdk is used in a setup,
        # which is not provided as a package by a Linux distribution.
        # Otherwise, dart-sdk is executed without the $SNAPSHOT_PATH
        # parameter, allowing to customize the dart-sdk provided as part
        # of the maintained package.
        exec "$DART" --packages="$FLUTTER_TOOLS_DIR/.dart_tool/package_config.json" $FLUTTER_TOOL_ARGS "$SNAPSHOT_PATH" "$@"
      else
        exec "$DART" --packages="$FLUTTER_TOOLS_DIR/.dart_tool/package_config.json" $FLUTTER_TOOL_ARGS "$@"
      fi
      ;;
    dart*)
      exec "$DART" "$@"
      ;;
    *)
      >&2 echo "Error! Executable name $BIN_NAME not recognized!"
      exit 1
      ;;
  esac
}
