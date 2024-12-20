#!/bin/bash
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

### Runs a Fuchsia integration test from shell/platform/fuchsia/flutter/tests/integration.
###
### Usage:
###   $ENGINE_DIR/flutter/tools/fuchsia/devshell/run_integration_test <integration_test_folder_name>
###
### Arguments:
###   --skip-fuchsia-build: Skips configuring and building Fuchsia for the test.
###   --skip-fuchsia-emu: Skips starting the Fuchsia emulator for the test.
###   --runtime-mode: The runtime mode to build Flutter in.
###                   Valid values: [debug, profile, release]
###                   Default value: debug
###   --fuchsia-cpu: The architecture of the Fuchsia device to target.
###                  Valid values: [x64, arm64]
###                  Default value: x64
###   --unoptimized: Disables C++ compiler optimizations.

set -e  # Fail on any error.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"/lib/vars.sh || exit $?

ensure_fuchsia_dir
jiri_bin=$FUCHSIA_DIR/.jiri_root/bin
ensure_engine_dir
ensure_ninja

if [[ $# -lt 2 ]]
then
  echo -e "Usage: $0 <integration_test_name>"
fi

# This script currently requires running `fx serve`.
if [[ -z "$(pgrep -f 'package-tool')" ]]
then
  engine-error "This script currently requires running 'fx serve' first."
  exit 1
fi

# The first argument is always assumed to be the integration test name.
test_name=$1
shift # past argument

# Ensure we know about the test and look up its packages.
# The first package listed here should be the main package for the test
# (the package that gets passed to `ffx test run`).
# Note: You do not need to include oot_flutter_jit_runner-0.far, the script
# automatically publishes it.
test_packages=
case $test_name in
  embedder)
    test_packages=("flutter-embedder-test-0.far" "parent-view.far" "child-view.far")
    ;;
  text-input)
    test_packages=("text-input-test-0.far" "text-input-view.far")
    ;;
  touch-input)
    test_packages=("touch-input-test-0.far" "touch-input-view.far" "embedding-flutter-view.far")
    ;;
  mouse-input)
    test_packages=("mouse-input-test-0.far" "mouse-input-view.far")
    ;;
  *)
    engine-error "Unknown test name $test_name. You may need to add it to $0"
    exit 1
    ;;
esac

# Parse arguments.
skip_fuchsia_build=0
skip_fuchsia_emu=0
runtime_mode="debug"
compilation_mode="jit"
fuchsia_cpu="x64"
ninja_cmd="ninja"
unoptimized_flags=""
unoptimized_suffix=""
extra_gn_args=()
while [[ $# -gt 0 ]]; do
  case $1 in
    --skip-fuchsia-build)
      shift # past argument
      skip_fuchsia_build=1
      ;;
    --skip-fuchsia-emu|--skip-fuchsia-emulator)
      shift # past argument
      skip_fuchsia_emu=1
      ;;
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

headless_flags=
if [[ -z "$DISPLAY" ]]
then
  engine-warning "You are running a Flutter integration test from a headless environment."
  engine-warning "This may lead to bugs or the test failing."
  engine-warning "You may want to switch to a graphical environment and try again, but the script will keep going."

  headless_flags="--headless"
fi

all_gn_args="--fuchsia --fuchsia-cpu="${fuchsia_cpu}" --runtime-mode="${runtime_mode}" ${unoptimized_flags} ${extra_gn_args[@]}"
engine-info "Building Flutter test with GN args: ${all_gn_args}"

"$ENGINE_DIR"/flutter/tools/gn ${all_gn_args}

fuchsia_out_dir_name=fuchsia_${runtime_mode}${unoptimized_suffix}_${fuchsia_cpu}
fuchsia_out_dir="$ENGINE_DIR"/out/"${fuchsia_out_dir_name}"
engine-info "Building ${fuchsia_out_dir_name}..."
${ninja_cmd} -C "${fuchsia_out_dir}" flutter/shell/platform/fuchsia/flutter/tests/integration/$test_name:tests

engine-debug "Printing test package contents for debugging..."
far_tool="$ENGINE_DIR"/fuchsia/sdk/linux/tools/x64/far
for test_package in "${test_packages[@]}"
do
  far_debug_dir=/tmp/"$test_name"_package_contents
  "${far_tool}" extract --archive="$(find $fuchsia_out_dir -name "$test_package")" --output="${far_debug_dir}"
  "${far_tool}" extract --archive="${far_debug_dir}"/meta.far --output="${far_debug_dir}"
  engine-debug "... $test_package tree:"
  tree "${far_debug_dir}"
  engine-debug "... $test_package/meta/contents:"
  cat "${far_debug_dir}"/meta/contents
  rm -r "${far_debug_dir}"
done

# .jiri_root/bin/ffx needs to run from $FUCHSIA_DIR.
pushd $FUCHSIA_DIR

engine-info "Registering debug symbols..."
"$FUCHSIA_DIR"/.jiri_root/bin/ffx debug symbol-index add "${fuchsia_out_dir}"/.build-id --build-dir "${fuchsia_out_dir}"

if [[ "$skip_fuchsia_build" -eq 0 ]]
then
  engine-info "Building Fuchsia in terminal.x64 mode... (to skip this, run with --skip-fuchsia-build)"
  if [[ "$runtime_mode" -eq "debug" ]]
  then
    "$jiri_bin"/fx set terminal.x64
  else
    "$jiri_bin"/fx set terminal.x64 --release
  fi
  "$jiri_bin"/fx build
fi

test_package_paths=( "$fuchsia_out_dir"/oot_flutter_jit_runner-0.far )
for test_package in "${test_packages[@]}"
do
  test_package_paths+=( $(find "$fuchsia_out_dir" -name "$test_package") )
done

fx_build_dir="$FUCHSIA_DIR/$(cat $FUCHSIA_DIR/.fx-build-dir)"
if [[ "$skip_fuchsia_emu" -eq 0 ]]
then
  engine-info "Starting the Fuchsia terminal.x64 emulator... (to skip this, run with --skip-fuchsia-emu)"
  "$jiri_bin"/ffx emu stop fuchsia-emulator
  "$jiri_bin"/ffx emu start "file://$fx_build_dir/*.json#terminal.x64" --net tap "${headless_flags}" --name fuchsia-emulator
fi

for test_package_path in "${test_package_paths[@]}"
do
  engine-info "... Publishing $test_package_path to package repository ($fx_build_dir/amber-files)..."
  "$jiri_bin"/ffx repository publish "$fx_build_dir/amber-files/" --package-archive "$test_package_path"
done

test_package_name_for_url="$(echo "${test_packages[0]}" | sed "s/\-0.far//")"
test_url="fuchsia-pkg://fuchsia.com/${test_package_name_for_url}/0#meta/${test_package_name_for_url}.cm"
engine-info "Running the test: $test_url"
"$jiri_bin"/ffx test run $test_url

popd  # $FUCHSIA_DIR
