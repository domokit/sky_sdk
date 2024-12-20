#!/bin/bash
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#
# This script expects ${ENGINE_PATH} to be set. It is currently used only
# by automation to collect and upload metrics.

set -ex

VARIANT=$1

${ENGINE_PATH}/src/out/${VARIANT}/txt_benchmarks --benchmark_format=json > ${ENGINE_PATH}/src/out/${VARIANT}/txt_benchmarks.json
${ENGINE_PATH}/src/out/${VARIANT}/fml_benchmarks --benchmark_format=json > ${ENGINE_PATH}/src/out/${VARIANT}/fml_benchmarks.json
${ENGINE_PATH}/src/out/${VARIANT}/shell_benchmarks --benchmark_format=json > ${ENGINE_PATH}/src/out/${VARIANT}/shell_benchmarks.json
${ENGINE_PATH}/src/out/${VARIANT}/ui_benchmarks --benchmark_format=json > ${ENGINE_PATH}/src/out/${VARIANT}/ui_benchmarks.json
${ENGINE_PATH}/src/out/${VARIANT}/display_list_builder_benchmarks --benchmark_format=json > ${ENGINE_PATH}/src/out/${VARIANT}/display_list_builder_benchmarks.json
${ENGINE_PATH}/src/out/${VARIANT}/display_list_region_benchmarks --benchmark_format=json > ${ENGINE_PATH}/src/out/${VARIANT}/display_list_region_benchmarks.json
${ENGINE_PATH}/src/out/${VARIANT}/display_list_transform_benchmarks --benchmark_format=json > ${ENGINE_PATH}/src/out/${VARIANT}/display_list_transform_benchmarks.json
${ENGINE_PATH}/src/out/${VARIANT}/geometry_benchmarks --benchmark_format=json > ${ENGINE_PATH}/src/out/${VARIANT}/geometry_benchmarks.json
