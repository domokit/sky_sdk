// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/embedder.h"

#include <set>

#include "flutter/testing/testing.h"

#ifdef _WIN32
// winbase.h defines GetCurrentTime as a macro.
#undef GetCurrentTime
#endif

namespace flutter {
namespace testing {

// Verifies that the proc table is fully populated.
TEST(EmbedderProcTable, AllPointersProvided) {
  FlutterEngineProcTable procs = {};
  procs.struct_size = sizeof(FlutterEngineProcTable);
  ASSERT_EQ(FlutterEngineGetProcAddresses(&procs), kSuccess);

  void (**proc)() = reinterpret_cast<void (**)()>(&procs.CreateAOTData);
  const uintptr_t end_address =
      reinterpret_cast<uintptr_t>(&procs) + procs.struct_size;
  while (reinterpret_cast<uintptr_t>(proc) < end_address) {
    EXPECT_NE(*proc, nullptr);
    ++proc;
  }
}

// Ensures that there are no duplicate pointers in the proc table, to catch
// copy/paste mistakes when adding a new entry to FlutterEngineGetProcAddresses.
TEST(EmbedderProcTable, NoDuplicatePointers) {
  FlutterEngineProcTable procs = {};
  procs.struct_size = sizeof(FlutterEngineProcTable);
  ASSERT_EQ(FlutterEngineGetProcAddresses(&procs), kSuccess);

  void (**proc)() = reinterpret_cast<void (**)()>(&procs.CreateAOTData);
  const uintptr_t end_address =
      reinterpret_cast<uintptr_t>(&procs) + procs.struct_size;
  std::set<void (*)()> seen_procs;
  while (reinterpret_cast<uintptr_t>(proc) < end_address) {
    auto result = seen_procs.insert(*proc);
    EXPECT_TRUE(result.second);
    ++proc;
  }
}

// Spot-checks that calling one of the function pointers works.
TEST(EmbedderProcTable, CallProc) {
  FlutterEngineProcTable procs = {};
  procs.struct_size = sizeof(FlutterEngineProcTable);
  ASSERT_EQ(FlutterEngineGetProcAddresses(&procs), kSuccess);

  EXPECT_NE(procs.GetCurrentTime(), 0ULL);
}

}  // namespace testing
}  // namespace flutter
