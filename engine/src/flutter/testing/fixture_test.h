// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_FIXTURE_TEST_H_
#define FLUTTER_TESTING_FIXTURE_TEST_H_

#include "flutter/testing/dart_fixture.h"

namespace flutter::testing {

class FixtureTest : public DartFixture, public ThreadTest {
 public:
  // Uses the default filenames from the fixtures generator.
  FixtureTest();

  // Allows to customize the kernel, ELF and split ELF filenames.
  FixtureTest(std::string kernel_filename,
              std::string elf_filename,
              std::string elf_split_filename);

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(FixtureTest);
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_FIXTURE_TEST_H_
