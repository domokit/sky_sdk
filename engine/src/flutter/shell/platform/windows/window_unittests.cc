// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <array>

#include "flutter/shell/platform/windows/testing/mock_direct_manipulation.h"
#include "flutter/shell/platform/windows/testing/mock_text_input_manager.h"
#include "flutter/shell/platform/windows/testing/mock_window.h"
#include "flutter/shell/platform/windows/testing/mock_windows_proc_table.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using testing::_;
using testing::Eq;
using testing::InSequence;
using testing::Invoke;
using testing::Return;

namespace flutter {
namespace testing {

TEST(MockWindow, CreateDestroy) {
  MockWindow window;
  ASSERT_TRUE(TRUE);
}

TEST(MockWindow, GetDpiAfterCreate) {
  MockWindow window;
  ASSERT_TRUE(window.GetDpi() > 0);
}

TEST(MockWindow, VerticalScroll) {
  MockWindow window;
  const int scroll_amount = 10;
  // Vertical scroll should be passed along, adjusted for scroll tick size
  // and direction.
  EXPECT_CALL(window, OnScroll(0, -scroll_amount / 120.0,
                               kFlutterPointerDeviceKindMouse, 0))
      .Times(1);

  window.InjectWindowMessage(WM_MOUSEWHEEL, MAKEWPARAM(0, scroll_amount), 0);
}

TEST(MockWindow, OnImeCompositionCompose) {
  auto windows_proc_table = std::make_unique<MockWindowsProcTable>();
  auto* text_input_manager = new MockTextInputManager();
  std::unique_ptr<TextInputManager> text_input_manager_ptr(text_input_manager);
  MockWindow window(std::move(windows_proc_table),
                    std::move(text_input_manager_ptr));
  EXPECT_CALL(*text_input_manager, GetComposingString())
      .WillRepeatedly(
          Return(std::optional<std::u16string>(std::u16string(u"nihao"))));
  EXPECT_CALL(*text_input_manager, GetResultString())
      .WillRepeatedly(
          Return(std::optional<std::u16string>(std::u16string(u"`}"))));
  EXPECT_CALL(*text_input_manager, GetComposingCursorPosition())
      .WillRepeatedly(Return((int)0));

  EXPECT_CALL(window, OnComposeChange(std::u16string(u"nihao"), 0)).Times(1);
  EXPECT_CALL(window, OnComposeChange(std::u16string(u"`}"), 0)).Times(0);
  EXPECT_CALL(window, OnComposeCommit()).Times(0);
  ON_CALL(window, OnImeComposition)
      .WillByDefault(Invoke(&window, &MockWindow::CallOnImeComposition));
  EXPECT_CALL(window, OnImeComposition(_, _, _)).Times(1);

  // Send an IME_COMPOSITION event that contains just the composition string.
  window.InjectWindowMessage(WM_IME_COMPOSITION, 0, GCS_COMPSTR);
}

TEST(MockWindow, OnImeCompositionResult) {
  auto windows_proc_table = std::make_unique<MockWindowsProcTable>();
  auto* text_input_manager = new MockTextInputManager();
  std::unique_ptr<TextInputManager> text_input_manager_ptr(text_input_manager);
  MockWindow window(std::move(windows_proc_table),
                    std::move(text_input_manager_ptr));
  EXPECT_CALL(*text_input_manager, GetComposingString())
      .WillRepeatedly(
          Return(std::optional<std::u16string>(std::u16string(u"nihao"))));
  EXPECT_CALL(*text_input_manager, GetResultString())
      .WillRepeatedly(
          Return(std::optional<std::u16string>(std::u16string(u"`}"))));
  EXPECT_CALL(*text_input_manager, GetComposingCursorPosition())
      .WillRepeatedly(Return((int)0));

  EXPECT_CALL(window, OnComposeChange(std::u16string(u"nihao"), 0)).Times(0);
  EXPECT_CALL(window, OnComposeChange(std::u16string(u"`}"), 0)).Times(1);
  EXPECT_CALL(window, OnComposeCommit()).Times(1);
  ON_CALL(window, OnImeComposition)
      .WillByDefault(Invoke(&window, &MockWindow::CallOnImeComposition));
  EXPECT_CALL(window, OnImeComposition(_, _, _)).Times(1);

  // Send an IME_COMPOSITION event that contains just the result string.
  window.InjectWindowMessage(WM_IME_COMPOSITION, 0, GCS_RESULTSTR);
}

TEST(MockWindow, OnImeCompositionResultAndCompose) {
  auto windows_proc_table = std::make_unique<MockWindowsProcTable>();
  auto* text_input_manager = new MockTextInputManager();
  std::unique_ptr<TextInputManager> text_input_manager_ptr(text_input_manager);
  MockWindow window(std::move(windows_proc_table),
                    std::move(text_input_manager_ptr));

  // This situation is that Google Japanese Input finished composing "今日" in
  // "今日は" but is still composing "は".
  {
    InSequence dummy;
    EXPECT_CALL(*text_input_manager, GetResultString())
        .WillRepeatedly(
            Return(std::optional<std::u16string>(std::u16string(u"今日"))));
    EXPECT_CALL(*text_input_manager, GetComposingString())
        .WillRepeatedly(
            Return(std::optional<std::u16string>(std::u16string(u"は"))));
  }
  {
    InSequence dummy;
    EXPECT_CALL(window, OnComposeChange(std::u16string(u"今日"), 0)).Times(1);
    EXPECT_CALL(window, OnComposeCommit()).Times(1);
    EXPECT_CALL(window, OnComposeChange(std::u16string(u"は"), 0)).Times(1);
  }

  EXPECT_CALL(*text_input_manager, GetComposingCursorPosition())
      .WillRepeatedly(Return((int)0));

  ON_CALL(window, OnImeComposition)
      .WillByDefault(Invoke(&window, &MockWindow::CallOnImeComposition));
  EXPECT_CALL(window, OnImeComposition(_, _, _)).Times(1);

  // send an IME_COMPOSITION event that contains both the result string and the
  // composition string.
  window.InjectWindowMessage(WM_IME_COMPOSITION, 0,
                             GCS_COMPSTR | GCS_RESULTSTR);
}

TEST(MockWindow, OnImeCompositionClearChange) {
  auto windows_proc_table = std::make_unique<MockWindowsProcTable>();
  auto* text_input_manager = new MockTextInputManager();
  std::unique_ptr<TextInputManager> text_input_manager_ptr(text_input_manager);
  MockWindow window(std::move(windows_proc_table),
                    std::move(text_input_manager_ptr));
  EXPECT_CALL(window, OnComposeChange(std::u16string(u""), 0)).Times(1);
  EXPECT_CALL(window, OnComposeCommit()).Times(1);
  ON_CALL(window, OnImeComposition)
      .WillByDefault(Invoke(&window, &MockWindow::CallOnImeComposition));
  EXPECT_CALL(window, OnImeComposition(_, _, _)).Times(1);

  // send an IME_COMPOSITION event that contains both the result string and the
  // composition string.
  window.InjectWindowMessage(WM_IME_COMPOSITION, 0, 0);
}

TEST(MockWindow, HorizontalScroll) {
  MockWindow window;
  const int scroll_amount = 10;
  // Vertical scroll should be passed along, adjusted for scroll tick size.
  EXPECT_CALL(window, OnScroll(scroll_amount / 120.0, 0,
                               kFlutterPointerDeviceKindMouse, 0))
      .Times(1);

  window.InjectWindowMessage(WM_MOUSEHWHEEL, MAKEWPARAM(0, scroll_amount), 0);
}

TEST(MockWindow, MouseLeave) {
  MockWindow window;
  const double mouse_x = 10.0;
  const double mouse_y = 20.0;

  EXPECT_CALL(window, OnPointerMove(mouse_x, mouse_y,
                                    kFlutterPointerDeviceKindMouse, 0, 0))
      .Times(1);
  EXPECT_CALL(window, OnPointerLeave(mouse_x, mouse_y,
                                     kFlutterPointerDeviceKindMouse, 0))
      .Times(1);

  window.InjectWindowMessage(WM_MOUSEMOVE, 0, MAKELPARAM(mouse_x, mouse_y));
  window.InjectWindowMessage(WM_MOUSELEAVE, 0, 0);
}

TEST(MockWindow, KeyDown) {
  MockWindow window;
  EXPECT_CALL(window, OnKey(_, _, _, _, _, _, _)).Times(1);
  LPARAM lparam = CreateKeyEventLparam(42, false, false);
  // send a "Shift" key down event.
  window.InjectWindowMessage(WM_KEYDOWN, 16, lparam);
}

TEST(MockWindow, KeyUp) {
  MockWindow window;
  EXPECT_CALL(window, OnKey(_, _, _, _, _, _, _)).Times(1);
  LPARAM lparam = CreateKeyEventLparam(42, false, true);
  // send a "Shift" key up event.
  window.InjectWindowMessage(WM_KEYUP, 16, lparam);
}

TEST(MockWindow, SysKeyDown) {
  MockWindow window;
  EXPECT_CALL(window, OnKey(_, _, _, _, _, _, _)).Times(1);
  LPARAM lparam = CreateKeyEventLparam(42, false, false);
  // send a "Shift" key down event.
  window.InjectWindowMessage(WM_SYSKEYDOWN, 16, lparam);
}

TEST(MockWindow, SysKeyUp) {
  MockWindow window;
  EXPECT_CALL(window, OnKey(_, _, _, _, _, _, _)).Times(1);
  LPARAM lparam = CreateKeyEventLparam(42, false, true);
  // send a "Shift" key up event.
  window.InjectWindowMessage(WM_SYSKEYUP, 16, lparam);
}

TEST(MockWindow, KeyDownPrintable) {
  MockWindow window;
  LPARAM lparam = CreateKeyEventLparam(30, false, false);

  auto respond_false = [](int key, int scancode, int action, char32_t character,
                          bool extended, bool was_down,
                          std::function<void(bool)> callback) {
    callback(false);
  };
  EXPECT_CALL(window, OnKey(65, 30, WM_KEYDOWN, 0, false, false, _))
      .Times(1)
      .WillOnce(respond_false);
  EXPECT_CALL(window, OnText(_)).Times(1);
  std::array<Win32Message, 2> messages = {
      Win32Message{WM_KEYDOWN, 65, lparam, kWmResultDontCheck},
      Win32Message{WM_CHAR, 65, lparam, kWmResultDontCheck}};
  window.InjectMessageList(2, messages.data());
}

TEST(MockWindow, KeyDownWithCtrl) {
  MockWindow window;

  // Simulate CONTROL pressed
  std::array<BYTE, 256> keyboard_state;
  keyboard_state[VK_CONTROL] = -1;
  SetKeyboardState(keyboard_state.data());

  LPARAM lparam = CreateKeyEventLparam(30, false, false);

  // Expect OnKey, but not OnText, because Control + Key is not followed by
  // WM_CHAR
  EXPECT_CALL(window, OnKey(65, 30, WM_KEYDOWN, 0, false, false, _)).Times(1);
  EXPECT_CALL(window, OnText(_)).Times(0);

  window.InjectWindowMessage(WM_KEYDOWN, 65, lparam);

  keyboard_state.fill(0);
  SetKeyboardState(keyboard_state.data());
}

TEST(MockWindow, KeyDownWithCtrlToggled) {
  MockWindow window;

  auto respond_false = [](int key, int scancode, int action, char32_t character,
                          bool extended, bool was_down,
                          std::function<void(bool)> callback) {
    callback(false);
  };

  // Simulate CONTROL toggled
  std::array<BYTE, 256> keyboard_state;
  keyboard_state[VK_CONTROL] = 1;
  SetKeyboardState(keyboard_state.data());

  LPARAM lparam = CreateKeyEventLparam(30, false, false);

  EXPECT_CALL(window, OnKey(65, 30, WM_KEYDOWN, 0, false, false, _))
      .Times(1)
      .WillOnce(respond_false);
  EXPECT_CALL(window, OnText(_)).Times(1);

  // send a "A" key down event.
  Win32Message messages[] = {{WM_KEYDOWN, 65, lparam, kWmResultDontCheck},
                             {WM_CHAR, 65, lparam, kWmResultDontCheck}};
  window.InjectMessageList(2, messages);

  keyboard_state.fill(0);
  SetKeyboardState(keyboard_state.data());
}

TEST(MockWindow, Paint) {
  MockWindow window;
  EXPECT_CALL(window, OnPaint()).Times(1);
  window.InjectWindowMessage(WM_PAINT, 0, 0);
}

// Verify direct manipulation isn't notified of pointer hit tests.
TEST(MockWindow, PointerHitTest) {
  UINT32 pointer_id = 123;
  auto windows_proc_table = std::make_unique<MockWindowsProcTable>();
  auto text_input_manager = std::make_unique<MockTextInputManager>();

  EXPECT_CALL(*windows_proc_table, GetPointerType(Eq(pointer_id), _))
      .Times(1)
      .WillOnce([](UINT32 pointer_id, POINTER_INPUT_TYPE* type) {
        *type = PT_POINTER;
        return TRUE;
      });

  MockWindow window(std::move(windows_proc_table),
                    std::move(text_input_manager));

  auto direct_manipulation =
      std::make_unique<MockDirectManipulationOwner>(&window);

  EXPECT_CALL(*direct_manipulation, SetContact).Times(0);

  window.SetDirectManipulationOwner(std::move(direct_manipulation));
  window.InjectWindowMessage(DM_POINTERHITTEST, MAKEWPARAM(pointer_id, 0), 0);
}

// Verify direct manipulation is notified of touchpad hit tests.
TEST(MockWindow, TouchPadHitTest) {
  UINT32 pointer_id = 123;
  auto windows_proc_table = std::make_unique<MockWindowsProcTable>();
  auto text_input_manager = std::make_unique<MockTextInputManager>();

  EXPECT_CALL(*windows_proc_table, GetPointerType(Eq(pointer_id), _))
      .Times(1)
      .WillOnce([](UINT32 pointer_id, POINTER_INPUT_TYPE* type) {
        *type = PT_TOUCHPAD;
        return TRUE;
      });

  MockWindow window(std::move(windows_proc_table),
                    std::move(text_input_manager));

  auto direct_manipulation =
      std::make_unique<MockDirectManipulationOwner>(&window);

  EXPECT_CALL(*direct_manipulation, SetContact(Eq(pointer_id))).Times(1);

  window.SetDirectManipulationOwner(std::move(direct_manipulation));
  window.InjectWindowMessage(DM_POINTERHITTEST, MAKEWPARAM(pointer_id, 0), 0);
}

// Verify direct manipulation isn't notified of unknown hit tests.
// This can happen if determining the pointer type fails, for example,
// if GetPointerType is unsupported by the current Windows version.
// See: https://github.com/flutter/flutter/issues/109412
TEST(MockWindow, UnknownPointerTypeSkipsDirectManipulation) {
  UINT32 pointer_id = 123;
  auto windows_proc_table = std::make_unique<MockWindowsProcTable>();
  auto text_input_manager = std::make_unique<MockTextInputManager>();

  EXPECT_CALL(*windows_proc_table, GetPointerType(Eq(pointer_id), _))
      .Times(1)
      .WillOnce(
          [](UINT32 pointer_id, POINTER_INPUT_TYPE* type) { return FALSE; });

  MockWindow window(std::move(windows_proc_table),
                    std::move(text_input_manager));

  auto direct_manipulation =
      std::make_unique<MockDirectManipulationOwner>(&window);

  EXPECT_CALL(*direct_manipulation, SetContact).Times(0);

  window.SetDirectManipulationOwner(std::move(direct_manipulation));
  window.InjectWindowMessage(DM_POINTERHITTEST, MAKEWPARAM(pointer_id, 0), 0);
}

// Test that the root UIA object is queried by WM_GETOBJECT.
TEST(MockWindow, DISABLED_GetObjectUia) {
  MockWindow window;
  bool uia_called = false;
  ON_CALL(window, OnGetObject)
      .WillByDefault(Invoke([&uia_called](UINT msg, WPARAM wpar, LPARAM lpar) {
#ifdef FLUTTER_ENGINE_USE_UIA
        uia_called = true;
#endif  // FLUTTER_ENGINE_USE_UIA
        return static_cast<LRESULT>(0);
      }));
  EXPECT_CALL(window, OnGetObject).Times(1);

  window.InjectWindowMessage(WM_GETOBJECT, 0, UiaRootObjectId);

  EXPECT_TRUE(uia_called);
}

}  // namespace testing
}  // namespace flutter
