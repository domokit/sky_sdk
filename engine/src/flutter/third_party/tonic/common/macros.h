// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef TONIC_COMMON_MACROS_H_
#define TONIC_COMMON_MACROS_H_

#include <cassert>
#include <cstdio>
#include <cstdlib>

#include "tonic/common/log.h"

#define TONIC_DISALLOW_COPY(TypeName) TypeName(const TypeName&) = delete;

#define TONIC_DISALLOW_ASSIGN(TypeName) \
  void operator=(const TypeName&) = delete;

#define TONIC_DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TONIC_DISALLOW_COPY(TypeName)                  \
  TONIC_DISALLOW_ASSIGN(TypeName)

#define TONIC_CHECK(condition)                    \
  {                                               \
    if (!(condition)) {                           \
      tonic::Log("assertion failed " #condition); \
      abort();                                    \
    }                                             \
  }

#ifndef NDEBUG
#define TONIC_DCHECK TONIC_CHECK
#else  // NDEBUG
#define TONIC_DCHECK (void)
#endif  // NDEBUG

#endif  // TONIC_COMMON_MACROS_H_
