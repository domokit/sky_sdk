#!/bin/bash
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#
### Builds and copies the Flutter and Dart runners for the Fuchsia platform.
###
### Arguments:
###   --runtime-mode: The runtime mode to build Flutter in.
###                   Valid values: [debug, profile, release]
###                   Default value: debug
###   --fuchsia-cpu: The architecture of the Fuchsia device to target.
###                  Valid values: [x64, arm64]
###                  Default value: x64
###   --unoptimized: Disables C++ compiler optimizations.
###
### Any additional arguments are forwarded directly to GN.

set -e  # Fail on any error.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"/lib/vars.sh || exit $?

ensure_fuchsia_dir
ensure_engine_dir
ensure_ninja

# Parse arguments.
runtime_mode="debug"
compilation_mode="jit"
fuchsia_cpu="x64"
ninja_cmd="ninja"
unoptimized_flags=""
unoptimized_suffix=""
extra_gn_args=()
while [[ $# -gt 0 ]]; do
  case $1 in
    --runtime-mode)
      shift # past argument
      runtime_mode="$1"
      shift # past value

      if [[ "${runtime_mode}" == debug ]]
      then
        compilation_mode="jit"
      elif [[ "${runtime_mode}" == profile || "${runtime_mode}" == release ]]
      then
        compilation_mode="aot"
      else
        engine-error "Invalid value for --runtime_mode: ${runtime_mode}"
        exit 1
      fi
      ;;
    --fuchsia-cpu)
      shift # past argument
      fuchsia_cpu="$1"
      shift # past value

      if [[ "${fuchsia_cpu}" != x64 && "${fuchsia_cpu}" != arm64 ]]
      then
        engine-error "Invalid value for --fuchsia-cpu: ${fuchsia_cpu}"
        exit 1
      fi
      ;;
    --unopt|--unoptimized)
      unoptimized_flags="--unoptimized"
      unoptimized_suffix="_unopt"
      shift # past argument
      ;;
    *)
      extra_gn_args+=("$1") # forward argument
      shift # past argument
      ;;
  esac
done

fuchsia_flutter_git_revision="$(cat $FUCHSIA_DIR/integration/jiri.lock | grep -A 1 "\"package\": \"flutter/fuchsia\"" | grep "git_revision" | tr ":" "\n" | sed -n 3p | tr "\"" "\n" | sed -n 1p)"
current_flutter_git_revision="$(git -C $ENGINE_DIR/flutter rev-parse HEAD)"
if [[ $fuchsia_flutter_git_revision != $current_flutter_git_revision ]]
then
  engine-warning "Your current Flutter Engine commit ($current_flutter_git_revision) is not Fuchsia's Flutter Engine commit ($fuchsia_flutter_git_revision)."
  engine-warning "You should checkout Fuchsia's Flutter Engine commit. This avoids crashing on app startup from using a different version of the Dart SDK. See https://github.com/flutter/flutter/wiki/Compiling-the-engine#important-dart-version-synchronization-on-fuchsia for more details."
  engine-warning "You can checkout Fuchsia's Flutter Engine commit by running:"
  engine-warning '$ENGINE_DIR/flutter/tools/fuchsia/devshell/checkout_fuchsia_revision.sh'
  engine-warning "If you have already checked out Fuchsia's Flutter Engine commit and then committed some additional changes, please ignore the above warning."
fi

all_gn_args="--fuchsia --fuchsia-cpu="${fuchsia_cpu}" --runtime-mode="${runtime_mode}" ${unoptimized_flags} ${extra_gn_args[@]}"
engine-info "GN args: ${all_gn_args}"

"$ENGINE_DIR"/flutter/tools/gn ${all_gn_args}

fuchsia_out_dir_name=fuchsia_${runtime_mode}${unoptimized_suffix}_${fuchsia_cpu}
fuchsia_out_dir="$ENGINE_DIR"/out/"${fuchsia_out_dir_name}"
engine-info "Building ${fuchsia_out_dir_name}..."
${ninja_cmd} -C "${fuchsia_out_dir}" flutter/shell/platform/fuchsia fuchsia_tests

engine-info "Making Fuchsia's Flutter prebuilts writable..."
chmod -R +w "$FUCHSIA_DIR"/prebuilt/third_party/flutter

engine-info "Copying the patched SDK (dart:ui, dart:zircon, dart:fuchsia) to Fuchsia..."
cp -ra "${fuchsia_out_dir}"/flutter_runner_patched_sdk/* "$FUCHSIA_DIR"/prebuilt/third_party/flutter/"${fuchsia_cpu}"/release/aot/flutter_runner_patched_sdk/

engine-info "Registering debug symbols..."
# .jiri_root/bin/ffx needs to run from $FUCHSIA_DIR.
pushd $FUCHSIA_DIR
"$FUCHSIA_DIR"/.jiri_root/bin/ffx debug symbol-index add "${fuchsia_out_dir}"/.build-id --build-dir "${fuchsia_out_dir}"
popd  # $FUCHSIA_DIR

if [[ "${runtime_mode}" == release ]]
then
  flutter_runner_pkg="flutter_jit_product_runner-0.far"
  engine-info "Copying the Flutter JIT product runner (${flutter_runner_pkg}) to Fuchsia..."
  cp "${fuchsia_out_dir}"/"${flutter_runner_pkg}" "$FUCHSIA_DIR"/prebuilt/third_party/flutter/"${fuchsia_cpu}"/release/jit/"${flutter_runner_pkg}"

  flutter_runner_pkg="flutter_aot_product_runner-0.far"
  engine-info "Copying the Flutter AOT product runner (${flutter_runner_pkg}) to Fuchsia..."
  cp "${fuchsia_out_dir}"/"${flutter_runner_pkg}" "$FUCHSIA_DIR"/prebuilt/third_party/flutter/"${fuchsia_cpu}"/release/aot/"${flutter_runner_pkg}"

  dart_runner_pkg="dart_jit_product_runner-0.far"
  engine-info "Copying the Dart JIT product runner (${dart_runner_pkg}) to Fuchsia..."
  cp "${fuchsia_out_dir}"/"${dart_runner_pkg}" "$FUCHSIA_DIR"/prebuilt/third_party/flutter/"${fuchsia_cpu}"/release/jit/"${dart_runner_pkg}"

  dart_runner_pkg="dart_aot_product_runner-0.far"
  engine-info "Copying the Dart AOT product runner (${dart_runner_pkg}) to Fuchsia..."
  cp "${fuchsia_out_dir}"/"${dart_runner_pkg}" "$FUCHSIA_DIR"/prebuilt/third_party/flutter/"${fuchsia_cpu}"/release/aot/"${dart_runner_pkg}"
else
  flutter_runner_pkg="flutter_${compilation_mode}_runner-0.far"
  engine-info "Copying the Flutter runner (${flutter_runner_pkg}) to Fuchsia..."
  cp "${fuchsia_out_dir}"/"${flutter_runner_pkg}" "$FUCHSIA_DIR"/prebuilt/third_party/flutter/"${fuchsia_cpu}"/"${runtime_mode}"/"${compilation_mode}"/"${flutter_runner_pkg}"

  dart_runner_pkg="dart_${compilation_mode}_runner-0.far"
  engine-info "Copying the Dart runner (${dart_runner_pkg}) to Fuchsia..."
  cp "${fuchsia_out_dir}"/"${dart_runner_pkg}" "$FUCHSIA_DIR"/prebuilt/third_party/flutter/"${fuchsia_cpu}"/"${runtime_mode}"/"${compilation_mode}"/"${dart_runner_pkg}"
fi

# TODO(akbiggs): Warn the developer when their current
# Fuchsia configuration (`fx set`) is mismatched with the runner
# they just deployed.

# TODO(akbiggs): Copy the tests over. I couldn't figure out a glob that grabs all of them.

echo "Done. You can now build Fuchsia with your Flutter Engine changes by running:"
echo '  cd $FUCHSIA_DIR'
# TODO(akbiggs): I'm not sure what example to give for arm64.
if [[ "${fuchsia_cpu}" == x64 ]]
then
  if [[ "${runtime_mode}" == debug ]]
  then
    echo "  fx set terminal.x64"
  else
    echo "  fx set terminal.x64 --release"
  fi
fi
echo '  fx build'
