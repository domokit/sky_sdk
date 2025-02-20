#!/usr/bin/env bash
# Copyright 2014 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# TODO(zanderso): refactor this and xcode_backend.sh into one script
# once iOS is using 'assemble'.
RunCommand() {
  if [[ -n "$VERBOSE_SCRIPT_LOGGING" ]]; then
    echo "♦ $*"
  fi
  "$@"
  return $?
}

EchoError() {
  echo "$@" 1>&2
}

ParseFlutterBuildMode() {
  # Use FLUTTER_BUILD_MODE if it's set, otherwise use the Xcode build configuration name
  # This means that if someone wants to use an Xcode build config other than Debug/Profile/Release,
  # they _must_ set FLUTTER_BUILD_MODE so we know what type of artifact to build.
  local build_mode="$(echo "${FLUTTER_BUILD_MODE:-${CONFIGURATION}}" | tr "[:upper:]" "[:lower:]")"

  case "$build_mode" in
    *release*) build_mode="release";;
    *profile*) build_mode="profile";;
    *debug*) build_mode="debug";;
    *)
      EchoError "========================================================================"
      EchoError "ERROR: Unknown FLUTTER_BUILD_MODE: ${build_mode}."
      EchoError "Valid values are 'Debug', 'Profile', or 'Release' (case insensitive)."
      EchoError "This is controlled by the FLUTTER_BUILD_MODE environment variable."
      EchoError "If that is not set, the CONFIGURATION environment variable is used."
      EchoError ""
      EchoError "You can fix this by either adding an appropriately named build"
      EchoError "configuration, or adding an appropriate value for FLUTTER_BUILD_MODE to the"
      EchoError ".xcconfig file for the current build configuration (${CONFIGURATION})."
      EchoError "========================================================================"
      exit -1;;
  esac
  echo "${build_mode}"
}

BuildApp() {
  # Set the working directory to the project root
  local project_path="${SOURCE_ROOT}/.."
  RunCommand pushd "${project_path}" > /dev/null

  # Set the target file.
  local target_path="lib/main.dart"
  if [[ -n "$FLUTTER_TARGET" ]]; then
      target_path="${FLUTTER_TARGET}"
  fi

  # Use FLUTTER_BUILD_MODE if it's set, otherwise use the Xcode build configuration name
  # This means that if someone wants to use an Xcode build config other than Debug/Profile/Release,
  # they _must_ set FLUTTER_BUILD_MODE so we know what type of artifact to build.
  local build_mode="$(ParseFlutterBuildMode)"

  if [[ -n "$LOCAL_ENGINE" ]]; then
    if [[ $(echo "$LOCAL_ENGINE" | tr "[:upper:]" "[:lower:]") != *"$build_mode"* ]]; then
      EchoError "========================================================================"
      EchoError "ERROR: Requested build with Flutter local engine at '${LOCAL_ENGINE}'"
      EchoError "This engine is not compatible with FLUTTER_BUILD_MODE: '${build_mode}'."
      EchoError "You can fix this by updating the LOCAL_ENGINE environment variable, or"
      EchoError "by running:"
      EchoError "  flutter build macos --local-engine=host_${build_mode} --local-engine-host=host_${build_mode}"
      EchoError "or"
      EchoError "  flutter build macos --local-engine=host_${build_mode}_unopt --local-engine-host=host_${build_mode}_unopt"
      EchoError "========================================================================"
      exit -1
    fi
  fi
  if [[ -n "$LOCAL_ENGINE_HOST" ]]; then
    if [[ $(echo "$LOCAL_ENGINE_HOST" | tr "[:upper:]" "[:lower:]") != *"$build_mode"* ]]; then
      EchoError "========================================================================"
      EchoError "ERROR: Requested build with Flutter local engine at '${LOCAL_ENGINE_HOST}'"
      EchoError "This engine is not compatible with FLUTTER_BUILD_MODE: '${build_mode}'."
      EchoError "You can fix this by updating the LOCAL_ENGINE_HOST environment variable, or"
      EchoError "by running:"
      EchoError "  flutter build macos --local-engine=host_${build_mode} --local-engine-host=host_${build_mode}"
      EchoError "or"
      EchoError "  flutter build macos --local-engine=host_${build_mode}_unopt --local-engine-host=host_${build_mode}_unopt"
      EchoError "========================================================================"
      exit -1
    fi
  fi

  # The path where the input/output xcfilelists are stored. These are used by xcode
  # to conditionally skip this script phase if neither have changed.
  local ephemeral_dir="${SOURCE_ROOT}/Flutter/ephemeral"
  local build_inputs_path="${ephemeral_dir}/FlutterInputs.xcfilelist"
  local build_outputs_path="${ephemeral_dir}/FlutterOutputs.xcfilelist"

  # Construct the "flutter assemble" argument array. Arguments should be added
  # as quoted string elements of the flutter_args array, otherwise an argument
  # (like a path) with spaces in it might be interpreted as two separate
  # arguments.
  local flutter_args=("${FLUTTER_ROOT}/bin/flutter")
  if [[ -n "$VERBOSE_SCRIPT_LOGGING" ]]; then
    flutter_args+=('--verbose')
  fi
  if [[ -n "$FLUTTER_ENGINE" ]]; then
    flutter_args+=("--local-engine-src-path=${FLUTTER_ENGINE}")
  fi
  if [[ -n "$LOCAL_ENGINE" ]]; then
    flutter_args+=("--local-engine=${LOCAL_ENGINE}")
  fi
  if [[ -n "$LOCAL_ENGINE_HOST" ]]; then
    flutter_args+=("--local-engine-host=${LOCAL_ENGINE_HOST}")
  fi

  local architectures="${ARCHS}"
  if [[ -n "$1" && "$1" == "prepare" ]]; then
    # The "prepare" command runs in a pre-action script, which doesn't always
    # filter the "ARCHS" build setting to only the active arch. To workaround,
    # if "ONLY_ACTIVE_ARCH" is true and the "NATIVE_ARCH" is arm, assume the
    # active arch is also arm to improve caching. If this assumption is
    # incorrect, it will later be corrected by the "build" command.
    if [[ -n "$ONLY_ACTIVE_ARCH" && "$ONLY_ACTIVE_ARCH" == "YES" && -n "$NATIVE_ARCH" ]]; then
      if [[ "$NATIVE_ARCH" == *"arm"*  ]]; then
        architectures="arm64"
      else
        architectures="x86_64"
      fi
    fi
  fi

  flutter_args+=(
    "assemble"
    "--no-version-check"
    "-dTargetPlatform=darwin"
    "-dDarwinArchs=${architectures}"
    "-dTargetFile=${target_path}"
    "-dBuildMode=${build_mode}"
    "-dTreeShakeIcons=${TREE_SHAKE_ICONS}"
    "-dDartObfuscation=${DART_OBFUSCATION}"
    "-dSplitDebugInfo=${SPLIT_DEBUG_INFO}"
    "-dTrackWidgetCreation=${TRACK_WIDGET_CREATION}"
    "-dAction=${ACTION}"
    "-dFrontendServerStarterPath=${FRONTEND_SERVER_STARTER_PATH}"
    "-dConfiguration=${CONFIGURATION}"
    "--DartDefines=${DART_DEFINES}"
    "--ExtraGenSnapshotOptions=${EXTRA_GEN_SNAPSHOT_OPTIONS}"
    "--ExtraFrontEndOptions=${EXTRA_FRONT_END_OPTIONS}"
    "--build-inputs=${build_inputs_path}"
    "--build-outputs=${build_outputs_path}"
    "--output=${BUILT_PRODUCTS_DIR}"
  )

  local target="${build_mode}_macos_bundle_flutter_assets";
  if [[ -n "$1" && "$1" == "prepare" ]]; then
    # The "prepare" command only targets the UnpackMacOS target, which copies the
    # FlutterMacOS framework to the BUILT_PRODUCTS_DIR.
    target="${build_mode}_unpack_macos"

    # Use the PreBuildAction define flag to force the tool to use a different
    # filecache file for the "prepare" command. This will make the environment
    # buildPrefix for the "prepare" command unique from the "build" command.
    # This will improve caching since the "build" command has more target dependencies.
    flutter_args+=("-dPreBuildAction=PrepareFramework")
  fi

  # FLAVOR is set by the Flutter CLI in the Flutter/ephemeral/Flutter-Generated.xcconfig
  # file when the --flavor flag is used, so it may not always be present.
  if [[ -n "$FLAVOR" ]]; then
    flutter_args+=("-dFlavor=${FLAVOR}")
  fi
  if [[ -n "$PERFORMANCE_MEASUREMENT_FILE" ]]; then
    flutter_args+=("--performance-measurement-file=${PERFORMANCE_MEASUREMENT_FILE}")
  fi
  if [[ -n "$CODE_SIZE_DIRECTORY" ]]; then
    flutter_args+=("-dCodeSizeDirectory=${CODE_SIZE_DIRECTORY}")
  fi

  flutter_args+=("${target}")

  RunCommand "${flutter_args[@]}"
}

PrepareFramework() {
  # The "prepare" command runs in a pre-action script, which also runs when
  # using the Xcode/xcodebuild clean command. Skip if cleaning.
  if [[ $ACTION == "clean" ]]; then
    exit 0
  fi
  BuildApp "prepare"
}

# Adds the App.framework as an embedded binary, the flutter_assets as
# resources, and the native assets.
EmbedFrameworks() {
  # Embed App.framework from Flutter into the app (after creating the Frameworks directory
  # if it doesn't already exist).
  local xcode_frameworks_dir="${TARGET_BUILD_DIR}/${FRAMEWORKS_FOLDER_PATH}"
  RunCommand mkdir -p -- "${xcode_frameworks_dir}"
  RunCommand rsync -av --delete --filter "- .DS_Store" "${BUILT_PRODUCTS_DIR}/App.framework" "${xcode_frameworks_dir}"

  # Embed the actual FlutterMacOS.framework that the Flutter app expects to run against,
  # which could be a local build or an arch/type specific build.

  # Copy Xcode behavior and don't copy over headers or modules.
  RunCommand rsync -av --delete --filter "- .DS_Store" --filter "- Headers" --filter "- Modules" "${BUILT_PRODUCTS_DIR}/FlutterMacOS.framework" "${xcode_frameworks_dir}/"

  # Sign the binaries we moved.
  if [[ -n "${EXPANDED_CODE_SIGN_IDENTITY:-}" ]]; then
    RunCommand codesign --force --verbose --sign "${EXPANDED_CODE_SIGN_IDENTITY}" -- "${xcode_frameworks_dir}/App.framework/App"
    RunCommand codesign --force --verbose --sign "${EXPANDED_CODE_SIGN_IDENTITY}" -- "${xcode_frameworks_dir}/FlutterMacOS.framework/FlutterMacOS"
  fi

  # Copy the native assets. These do not have to be codesigned here because,
  # they are already codesigned in buildNativeAssetsMacOS.
  local project_path="${SOURCE_ROOT}/.."
  if [[ -n "$FLUTTER_APPLICATION_PATH" ]]; then
      project_path="${FLUTTER_APPLICATION_PATH}"
  fi
  local native_assets_path="${project_path}/${FLUTTER_BUILD_DIR}/native_assets/macos/"
  if [[ -d "$native_assets_path" ]]; then
    RunCommand rsync -av --filter "- .DS_Store" --filter "- native_assets.yaml" --filter "- native_assets.json" "${native_assets_path}" "${xcode_frameworks_dir}"

    # Iterate through all .frameworks in native assets directory.
    for native_asset in "${native_assets_path}"*.framework; do
      [ -e "$native_asset" ] || continue # Skip when there are no matches.
      # Codesign the framework inside the app bundle.
      RunCommand codesign --force --verbose --sign "${EXPANDED_CODE_SIGN_IDENTITY}" -- "${xcode_frameworks_dir}/$(basename "$native_asset")"
    done
  fi
}

# Main entry point.
if [[ $# == 0 ]]; then
  # Unnamed entry point defaults to build.
  BuildApp
else
  case $1 in
    "build")
      BuildApp ;;
    "embed")
      EmbedFrameworks ;;
    "prepare")
      PrepareFramework ;;
  esac
fi
