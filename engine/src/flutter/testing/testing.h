// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_TESTING_H_
#define FLUTTER_TESTING_TESTING_H_

#include <string>
#include <vector>

#include "flutter/fml/file.h"
#include "flutter/fml/mapping.h"
#include "flutter/testing/assertions.h"
#include "gtest/gtest.h"
#include "third_party/skia/include/core/SkData.h"

namespace flutter::testing {

const char* GetSourcePath();

//------------------------------------------------------------------------------
/// @brief      Returns the directory containing the test fixture for the target
///             if this target has fixtures configured. If there are no
///             fixtures, this is a link error. If you see a linker error on
///             this symbol, the unit-test target needs to depend on a
///             `test_fixtures` target.
///
/// @return     The fixtures path.
///
const char* GetFixturesPath();

//------------------------------------------------------------------------------
/// @brief      Returns the directory containing assets shared across all tests.
///
/// @return     The testing assets path.
///
const char* GetTestingAssetsPath();

//------------------------------------------------------------------------------
/// @brief      Returns the default path to kernel_blob.bin. This file is within
///             the directory returned by `GetFixturesPath()`.
///
/// @return     The kernel file path.
///
std::string GetDefaultKernelFilePath();

//------------------------------------------------------------------------------
/// @brief      Opens the fixtures directory for the unit-test harness.
///
/// @return     The file descriptor of the fixtures directory.
///
fml::UniqueFD OpenFixturesDirectory();

//------------------------------------------------------------------------------
/// @brief      Opens a fixture of the given file name.
///
/// @param[in]  fixture_name  The fixture name
///
/// @return     The file descriptor of the given fixture. An invalid file
///             descriptor is returned in case the fixture is not found.
///
fml::UniqueFD OpenFixture(const std::string& fixture_name);

//------------------------------------------------------------------------------
/// @brief      Opens a fixture of the given file name and returns a mapping to
///             its contents.
///
/// @param[in]  fixture_name  The fixture name
///
/// @return     A mapping to the contents of fixture or null if the fixture does
///             not exist or its contents cannot be mapped in.
///
std::unique_ptr<fml::Mapping> OpenFixtureAsMapping(
    const std::string& fixture_name);

//------------------------------------------------------------------------------
/// @brief      Opens a fixture of the given file name and returns a Skia SkData
///             holding its contents.
///
/// @param[in]  fixture_name  The fixture name
///
/// @return     An SkData, or null if the fixture does not exist or its contents
///             cannot be mapped in.
///
sk_sp<SkData> OpenFixtureAsSkData(const std::string& fixture_name);

//------------------------------------------------------------------------------
/// @brief      Gets the name of the currently running test. This is useful in
///             generating logs or assets based on test name.
///
/// @return     The current test name.
///
std::string GetCurrentTestName();

enum class MemsetPatternOp {
  kMemsetPatternOpSetBuffer,
  kMemsetPatternOpCheckBuffer,
};

//------------------------------------------------------------------------------
/// @brief      Depending on the operation, either scribbles a known pattern
///             into the buffer or checks if that pattern is present in an
///             existing buffer. This is a portable variant of the
///             memset_pattern class of methods that also happen to do assert
///             that the same pattern exists.
///
/// @param      buffer  The buffer
/// @param[in]  size    The size
/// @param[in]  op      The operation
///
/// @return     If the result of the operation was a success.
///
bool MemsetPatternSetOrCheck(uint8_t* buffer, size_t size, MemsetPatternOp op);

bool MemsetPatternSetOrCheck(std::vector<uint8_t>& buffer, MemsetPatternOp op);

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_TESTING_H_
