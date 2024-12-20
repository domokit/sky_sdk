#!/bin/bash

# Copyright (c) 2013 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# For app bundles built with ASan, copies the runtime lib
# (libclang_rt.asan_osx_dynamic.dylib), on which their executables depend, from
# the compiler installation path into the bundle and fixes the dylib's install
# name in the binary to be relative to @executable_path.

set -e

BINARY="${BUILT_PRODUCTS_DIR}/${EXECUTABLE_PATH}"

if [[ ! -f "$BINARY" ]]; then
  # This is neither an .app bundle nor a standalone executable.
  # Most certainly the script has been called for a data bundle.
  exit 0
fi

BINARY_DIR="$(dirname "${BINARY}")"

# Find the link to the ASan runtime encoded in the binary.
BUILTIN_DYLIB_PATH=$(otool -L "${BINARY}" | \
    sed -Ene 's/^[[:blank:]]+(.*libclang_rt\.asan_.*_dynamic\.dylib).*$/\1/p')

if [[ "${BUILTIN_DYLIB_PATH}" == *asan_iossim_dynamic* ]]; then
  ASAN_DYLIB_NAME=libclang_rt.asan_iossim_dynamic.dylib
elif [[ "${BUILTIN_DYLIB_PATH}" == *asan_osx_dynamic* ]]; then
  ASAN_DYLIB_NAME=libclang_rt.asan_osx_dynamic.dylib
fi

if [[ -z "${BUILTIN_DYLIB_PATH}" ]]; then
  echo "${BINARY} does not depend on the ASan runtime library!" >&2
  exit 1
fi

# TODO(glider): this doesn't work if we set CC and CXX to override the default
# Clang.
ASAN_DYLIB=$(find \
    "${BUILT_PRODUCTS_DIR}/../../third_party/llvm-build/Release+Asserts/lib/clang/" \
    -type f -path "*${ASAN_DYLIB_NAME}")

DYLIB_BASENAME=$(basename "${ASAN_DYLIB}")
if [[ "${DYLIB_BASENAME}" != "${ASAN_DYLIB_NAME}" ]]; then
  echo "basename(${ASAN_DYLIB}) != ${ASAN_DYLIB_NAME}" >&2
  exit 1
fi

# Check whether the directory containing the executable binary is named
# "MacOS". In this case we're building a full-fledged OSX app and will put
# the runtime into appname.app/Contents/Libraries/. Otherwise this is probably
# an iOS gtest app, and the ASan runtime is put next to the executable.
UPPER_DIR=$(dirname "${BINARY_DIR}")
if [ "${UPPER_DIR}" == "MacOS" ]; then
  LIBRARIES_DIR="${UPPER_DIR}/Libraries"
  mkdir -p "${LIBRARIES_DIR}"
  NEW_LC_ID_DYLIB="@executable_path/../Libraries/${ASAN_DYLIB_NAME}"
else
  LIBRARIES_DIR="${BINARY_DIR}"
  NEW_LC_ID_DYLIB="@executable_path/${ASAN_DYLIB_NAME}"
fi

cp "${ASAN_DYLIB}" "${LIBRARIES_DIR}"

# Make LC_ID_DYLIB of the runtime copy point to its location.
install_name_tool \
    -id "${NEW_LC_ID_DYLIB}" \
    "${LIBRARIES_DIR}/${ASAN_DYLIB_NAME}"

# Fix the rpath to the runtime library recorded in the binary.
install_name_tool \
    -change "${BUILTIN_DYLIB_PATH}" \
    "${NEW_LC_ID_DYLIB}" \
    "${BINARY}"
