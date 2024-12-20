// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef ACCESSIBILITY_BASE_COMPILER_SPECIFIC_H_
#define ACCESSIBILITY_BASE_COMPILER_SPECIFIC_H_

#if !defined(__GNUC__) && !defined(__clang__) && !defined(_MSC_VER)
#error Unsupported compiler.
#endif

// Annotate a variable indicating it's ok if the variable is not used.
// (Typically used to silence a compiler warning when the assignment
// is important for some other reason.)
// Use like:
//   int x = ...;
//   BASE_ALLOW_UNUSED_LOCAL(x);
#define BASE_ALLOW_UNUSED_LOCAL(x) false ? (void)x : (void)0

// Annotate a typedef or function indicating it's ok if it's not used.
// Use like:
//   typedef Foo Bar ALLOW_UNUSED_TYPE;
#if defined(__GNUC__) || defined(__clang__)
#define BASE_ALLOW_UNUSED_TYPE __attribute__((unused))
#else
#define BASE_ALLOW_UNUSED_TYPE
#endif

#endif  // ACCESSIBILITY_BASE_COMPILER_SPECIFIC_H_

// Macro for hinting that an expression is likely to be false.
#if !defined(BASE_UNLIKELY)
#if defined(COMPILER_GCC) || defined(__clang__)
#define BASE_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#define BASE_UNLIKELY(x) (x)
#endif  // defined(COMPILER_GCC)
#endif  // !defined(BASE_UNLIKELY)

#if !defined(BASE_LIKELY)
#if defined(COMPILER_GCC) || defined(__clang__)
#define BASE_LIKELY(x) __builtin_expect(!!(x), 1)
#else
#define BASE_LIKELY(x) (x)
#endif  // defined(COMPILER_GCC)
#endif  // !defined(BASE_LIKELY)

// Macro for telling -Wimplicit-fallthrough that a fallthrough is intentional.
#if defined(__clang__)
#define BASE_FALLTHROUGH [[clang::fallthrough]]
#else
#define BASE_FALLTHROUGH
#endif
