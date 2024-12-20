#!/bin/bash
set -e
set -x

# web_analysis: a command-line utility for running 'dart analyze' on Flutter Web
# Engine. Used/Called by LUCI recipes:
#
# See: https://flutter.googlesource.com/recipes/+/refs/heads/main/recipes/web_engine.py

echo "Engine path $ENGINE_PATH"
WEB_UI_DIR="$ENGINE_PATH/src/flutter/lib/web_ui"
DART_SDK_DIR="${ENGINE_PATH}/src/out/host_debug_unopt/dart-sdk"
DART_PATH="$DART_SDK_DIR/bin/dart"

echo "Running \`dart pub get\` in 'engine/src/flutter/lib/web_ui'"
(cd "$WEB_UI_DIR"; $DART_PATH pub --suppress-analytics get)

echo "Running \`dart analyze\` in 'engine/src/flutter/lib/web_ui'"
(cd "$WEB_UI_DIR"; $DART_PATH analyze --suppress-analytics --fatal-infos)
