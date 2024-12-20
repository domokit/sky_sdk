// Copyright 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_TEST_GTEST_UTIL_H_
#define BASE_TEST_GTEST_UTIL_H_

#include <string>
#include <utility>
#include <vector>

#include "base/compiler_specific.h"
#include "build/build_config.h"
#include "gtest/gtest.h"

// EXPECT/ASSERT_DCHECK_DEATH is intended to replace EXPECT/ASSERT_DEBUG_DEATH
// when the death is expected to be caused by a DCHECK. Contrary to
// EXPECT/ASSERT_DEBUG_DEATH however, it doesn't execute the statement in non-
// dcheck builds as DCHECKs are intended to catch things that should never
// happen and as such executing the statement results in undefined behavior
// (|statement| is compiled in unsupported configurations nonetheless).
// Death tests misbehave on Android.
// TODO(gw280): once https://github.com/flutter/flutter/issues/78491 is resolved
// we can potentially remove the condition on NDEBUG here.
#if defined(GTEST_HAS_DEATH_TEST) && !defined(OS_ANDROID) && !defined(NDEBUG)

// EXPECT/ASSERT_DCHECK_DEATH tests verify that a DCHECK is hit ("Check failed"
// is part of the error message), but intentionally do not expose the gtest
// death test's full |regex| parameter to avoid users having to verify the exact
// syntax of the error message produced by the DCHECK.
#define EXPECT_DCHECK_DEATH(statement) EXPECT_DEATH(statement, "Check failed")
#define ASSERT_DCHECK_DEATH(statement) ASSERT_DEATH(statement, "Check failed")

#else
// defined(GTEST_HAS_DEATH_TEST) && !defined(OS_ANDROID) && !defined(NDEBUG)

#define EXPECT_DCHECK_DEATH(statement) \
  GTEST_UNSUPPORTED_DEATH_TEST(statement, "Check failed", )
#define ASSERT_DCHECK_DEATH(statement) \
  GTEST_UNSUPPORTED_DEATH_TEST(statement, "Check failed", return)

#endif
// defined(GTEST_HAS_DEATH_TEST) && !defined(OS_ANDROID) && !defined(NDEBUG)

// As above, but for CHECK().
#if defined(GTEST_HAS_DEATH_TEST) && !defined(OS_ANDROID)

// Official builds will eat stream parameters, so don't check the error message.
#if defined(OFFICIAL_BUILD) && defined(NDEBUG)
#define EXPECT_CHECK_DEATH(statement) EXPECT_DEATH(statement, "")
#define ASSERT_CHECK_DEATH(statement) ASSERT_DEATH(statement, "")
#else
#define EXPECT_CHECK_DEATH(statement) EXPECT_DEATH(statement, "Check failed")
#define ASSERT_CHECK_DEATH(statement) ASSERT_DEATH(statement, "Check failed")
#endif  // defined(OFFICIAL_BUILD) && defined(NDEBUG)

#else  // defined(GTEST_HAS_DEATH_TEST) && !defined(OS_ANDROID)

// Note GTEST_UNSUPPORTED_DEATH_TEST takes a |regex| only to see whether it is a
// valid regex. It is never evaluated.
#define EXPECT_CHECK_DEATH(statement) \
  GTEST_UNSUPPORTED_DEATH_TEST(statement, "", )
#define ASSERT_CHECK_DEATH(statement) \
  GTEST_UNSUPPORTED_DEATH_TEST(statement, "", return)

#endif  // defined(GTEST_HAS_DEATH_TEST) && !defined(OS_ANDROID)

#endif  // BASE_TEST_GTEST_UTIL_H_
