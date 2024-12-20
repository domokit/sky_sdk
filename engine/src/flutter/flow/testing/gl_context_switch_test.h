// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_TESTING_GL_CONTEXT_SWITCH_TEST_H_
#define FLUTTER_FLOW_TESTING_GL_CONTEXT_SWITCH_TEST_H_

#include "flutter/common/graphics/gl_context_switch.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

//------------------------------------------------------------------------------
/// The renderer context used for testing
class TestSwitchableGLContext : public SwitchableGLContext {
 public:
  explicit TestSwitchableGLContext(int context);

  ~TestSwitchableGLContext() override;

  bool SetCurrent() override;

  bool RemoveCurrent() override;

  int GetContext();

  static int GetCurrentContext();

  //------------------------------------------------------------------------------
  /// Set the current context
  ///
  /// This is to mimic how other programs outside flutter sets the context.
  static void SetCurrentContext(int context);

 private:
  int context_;

  FML_DISALLOW_COPY_AND_ASSIGN(TestSwitchableGLContext);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_FLOW_TESTING_GL_CONTEXT_SWITCH_TEST_H_
