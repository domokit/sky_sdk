// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_TEST_UTILS_PROC_TABLE_REPLACEMENT_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_TEST_UTILS_PROC_TABLE_REPLACEMENT_H_

#include "flutter/shell/platform/embedder/embedder.h"

// Wraps capturing lambas with non-capturing version that can be assigned to
// FlutterEngineProcTable entries (by using statics) to facilitate mocking in
// tests of code built on top of the embedder API.
//
// This should *ONLY* be used in unit tests as it is leaky by design.
//
// |proc| should be the name of an entry in FlutterEngineProcTable, such as
// "Initialize". |mock_impl| should be a lamba that replaces its implementation,
// taking the same arguments and returning the same type.
#define MOCK_ENGINE_PROC(proc, mock_impl)                                      \
  ([&]() {                                                                     \
    static std::function<                                                      \
        std::remove_pointer_t<decltype(FlutterEngineProcTable::proc)>>         \
        closure;                                                               \
    closure = mock_impl;                                                       \
    static auto non_capturing = [](auto... args) { return closure(args...); }; \
    return non_capturing;                                                      \
  })()

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_TEST_UTILS_PROC_TABLE_REPLACEMENT_H_
