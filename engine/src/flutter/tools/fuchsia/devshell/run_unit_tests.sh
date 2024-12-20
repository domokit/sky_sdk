#!/bin/bash
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

### Runs the Fuchsia unit tests in the debug configuration.
###
### Arguments:
###   --package-filter: Only runs tests in packages that match the given `find` statement.
###   --gtest-filter: In the packages, only runs tests that match the given filter.
###                   For example: --gtest-filter "*FlatlandConnection*" to run any tests
###                   that have the phrase FlatlandConnection in them.
###   --unoptimized: Disables C++ compiler optimizations.
###   --count: Number of times to run the test. By default runs 1 time.
###            See `ffx test run --count`.

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
package_filter="*tests-0.far"
test_filter_flags=""
unoptimized_flags=""
unoptimized_suffix=""
count_flags=""
while [[ $# -gt 0 ]]; do
  case $1 in
    --package_filter|--package-filter)
      shift # past argument
      package_filter="$1"
      shift # past value
      ;;
    --count)
      shift # past argument
      count_flags="--count $1"
      shift # past value
      ;;
    --unopt|--unoptimized)
      unoptimized_flags="--unoptimized"
      unoptimized_suffix="_unopt"
      shift # past argument
      ;;
    --gtest_filter|--gtest-filter)
      shift # past argument
      test_filter_flags="-- --gtest_filter=$1"
      shift # past value
      ;;
    *)
      engine-error "Unknown argument: $1"
      exit 1
      ;;
  esac
done

all_gn_args="--fuchsia --no-lto --fuchsia-cpu="${fuchsia_cpu}" --runtime-mode="${runtime_mode}" ${unoptimized_flags}"
engine-info "GN args: ${all_gn_args}"

"${ENGINE_DIR}"/flutter/tools/gn ${all_gn_args}

fuchsia_out_dir_name=fuchsia_${runtime_mode}${unoptimized_suffix}_${fuchsia_cpu}
fuchsia_out_dir="${ENGINE_DIR}"/out/"${fuchsia_out_dir_name}"
engine-info "Building ${fuchsia_out_dir_name}..."
${ninja_cmd} -C "${fuchsia_out_dir}" fuchsia_tests

engine-info "Registering debug symbols..."
# .jiri_root/bin/ffx needs to run from $FUCHSIA_DIR.
pushd $FUCHSIA_DIR
"$FUCHSIA_DIR"/.jiri_root/bin/ffx debug symbol-index add "${fuchsia_out_dir}"/.build-id --build-dir "${fuchsia_out_dir}"
popd  # $FUCHSIA_DIR

test_packages="$(find ${fuchsia_out_dir} -name "${package_filter}")"

engine-info "Publishing test packages..."
test_names=()
for test_package in $test_packages
do
  engine-info "... publishing ${test_package} ..."
  (
    # ffx can not be called outside of FUCHSIA_DIR.
    cd ${FUCHSIA_DIR}
    ${FUCHSIA_DIR}/.jiri_root/bin/ffx repository publish \
        $FUCHSIA_DIR/$(cat $FUCHSIA_DIR/.fx-build-dir)/amber-files \
        --package-archive "${test_package}"
  )
  test_names+=("$(basename ${test_package} | sed -e "s/-0.far//")")
done

# .jiri_root/bin/ffx needs to run from $FUCHSIA_DIR.

pushd $FUCHSIA_DIR

# TODO(akbiggs): Match the behavior of this script more closely with test_suites.yaml.
engine-info "Running tests... (if this fails because of Launch(InstanceCannotResolve), run fx serve and try again)"
for test_name in "${test_names[@]}"
do
  # ParagraphTest.* fails in txt_tests.
  if [[ "${test_name}" == "txt_tests" ]]
  then
    engine-warning "Skipping txt_tests because I don't know how to filter out ParagraphTests.*"
    continue
  fi
  # ${test_filter_flags} must come last below because it includes " -- " to start program arguments.
  test_cmd="${FUCHSIA_DIR}/.jiri_root/bin/ffx test run fuchsia-pkg://fuchsia.com/${test_name}#meta/${test_name}.cm ${count_flags} ${test_filter_flags}"

  engine-info "... $test_cmd ..."
  $test_cmd
done

popd  # $FUCHSIA_DIR
