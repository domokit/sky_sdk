// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/fixture_test.h"

#include <utility>

#include "flutter/testing/dart_fixture.h"

namespace flutter::testing {

FixtureTest::FixtureTest() : DartFixture() {}

FixtureTest::FixtureTest(std::string kernel_filename,
                         std::string elf_filename,
                         std::string elf_split_filename)
    : DartFixture(std::move(kernel_filename),
                  std::move(elf_filename),
                  std::move(elf_split_filename)) {}

}  // namespace flutter::testing
