// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_TEXT_INPUT_VIEW_DELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_TEXT_INPUT_VIEW_DELEGATE_H_

#include <unordered_map>

#include "flutter/shell/platform/linux/fl_text_input_view_delegate.h"

#include "gmock/gmock.h"

namespace flutter {
namespace testing {

// Mock for FlTextInputVuewDelegate.
class MockTextInputViewDelegate {
 public:
  MockTextInputViewDelegate();
  ~MockTextInputViewDelegate();

  // NOLINTNEXTLINE(google-explicit-constructor)
  operator FlTextInputViewDelegate*();

  MOCK_METHOD(void,
              fl_text_input_view_delegate_translate_coordinates,
              (FlTextInputViewDelegate * delegate,
               gint view_x,
               gint view_y,
               gint* window_x,
               gint* window_y));

 private:
  FlTextInputViewDelegate* instance_ = nullptr;
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_TEXT_INPUT_VIEW_DELEGATE_H_
