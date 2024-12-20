// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_STRUCT_MACROS_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_STRUCT_MACROS_H_

#include <type_traits>

// Checks if the given struct contains a member, whether set or not.
#define STRUCT_HAS_MEMBER(pointer, member)                           \
  ((offsetof(std::remove_pointer<decltype(pointer)>::type, member) + \
        sizeof(pointer->member) <=                                   \
    pointer->struct_size))

#define SAFE_ACCESS(pointer, member, default_value)                 \
  ([=]() {                                                          \
    if (STRUCT_HAS_MEMBER(pointer, member)) {                       \
      return pointer->member;                                       \
    }                                                               \
    return static_cast<decltype(pointer->member)>((default_value)); \
  })()

/// Checks if the member exists and is non-null.
#define SAFE_EXISTS(pointer, member) \
  (SAFE_ACCESS(pointer, member, nullptr) != nullptr)

/// Checks if exactly one of member1 or member2 exists and is non-null.
#define SAFE_EXISTS_ONE_OF(pointer, member1, member2) \
  (SAFE_EXISTS(pointer, member1) != SAFE_EXISTS(pointer, member2))

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_STRUCT_MACROS_H_
