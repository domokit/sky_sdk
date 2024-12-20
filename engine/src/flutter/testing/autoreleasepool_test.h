// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_AUTORELEASEPOOL_TEST_H_
#define FLUTTER_TESTING_AUTORELEASEPOOL_TEST_H_

#include "flutter/fml/platform/darwin/scoped_nsautorelease_pool.h"

#include "gtest/gtest.h"

namespace flutter::testing {

// GoogleTest mixin that runs the test within the scope of an NSAutoReleasePool.
//
// This can be mixed into test fixture classes that also inherit from gtest's
// ::testing::Test base class.
class AutoreleasePoolTestMixin {
 public:
  AutoreleasePoolTestMixin() = default;
  ~AutoreleasePoolTestMixin() = default;

 private:
  fml::ScopedNSAutoreleasePool autorelease_pool_;

  FML_DISALLOW_COPY_AND_ASSIGN(AutoreleasePoolTestMixin);
};

// GoogleTest fixture that runs the test within the scope of an
// NSAutoReleasePool.
class AutoreleasePoolTest : public ::testing::Test,
                            public AutoreleasePoolTestMixin {
 public:
  AutoreleasePoolTest() = default;
  ~AutoreleasePoolTest() = default;

 private:
  fml::ScopedNSAutoreleasePool autorelease_pool_;

  FML_DISALLOW_COPY_AND_ASSIGN(AutoreleasePoolTest);
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_AUTORELEASEPOOL_TEST_H_
