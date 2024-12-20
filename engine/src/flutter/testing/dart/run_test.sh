#!/bin/bash

set -e

FILE=$1
COMPILE_TARGET=compile_$FILE.dart
DART_FILTER=$FILE.dart

ninja -C ../out/host_debug_unopt_arm64 $COMPILE_TARGET
./testing/run_tests.py --type=dart --dart-filter=$DART_FILTER --variant=host_debug_unopt_arm64
