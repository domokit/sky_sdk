// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_PLAYGROUND_TEST_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_PLAYGROUND_TEST_H_

#include <functional>

#include "impeller/playground/playground_test.h"
#include "impeller/toolkit/interop/context.h"
#include "impeller/toolkit/interop/surface.h"

namespace impeller::interop::testing {

class PlaygroundTest : public ::impeller::PlaygroundTest {
 public:
  PlaygroundTest();

  // |PlaygroundTest|
  ~PlaygroundTest() override;

  PlaygroundTest(const PlaygroundTest&) = delete;

  PlaygroundTest& operator=(const PlaygroundTest&) = delete;

  // |PlaygroundTest|
  void SetUp() override;

  // |PlaygroundTest|
  void TearDown() override;

  ScopedObject<Context> CreateContext() const;

  ScopedObject<Context> GetInteropContext();

  using InteropPlaygroundCallback =
      std::function<bool(const ScopedObject<Context>& context,
                         const ScopedObject<Surface>& surface)>;
  bool OpenPlaygroundHere(InteropPlaygroundCallback callback);

 private:
  ScopedObject<Context> interop_context_;
};

}  // namespace impeller::interop::testing

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_PLAYGROUND_TEST_H_
