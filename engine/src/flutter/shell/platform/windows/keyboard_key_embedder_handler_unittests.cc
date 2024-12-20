// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/keyboard_key_embedder_handler.h"

#include <string>
#include <vector>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/embedder/test_utils/key_codes.g.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/windows/keyboard_utils.h"
#include "flutter/shell/platform/windows/testing/engine_modifier.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

namespace {

constexpr SHORT kStateMaskToggled = 0x01;
constexpr SHORT kStateMaskPressed = 0x80;

class TestFlutterKeyEvent : public FlutterKeyEvent {
 public:
  TestFlutterKeyEvent(const FlutterKeyEvent& src,
                      FlutterKeyEventCallback callback,
                      void* user_data)
      : character_str(src.character), callback(callback), user_data(user_data) {
    struct_size = src.struct_size;
    timestamp = src.timestamp;
    type = src.type;
    physical = src.physical;
    logical = src.logical;
    character = character_str.c_str();
    synthesized = src.synthesized;
  }

  TestFlutterKeyEvent(TestFlutterKeyEvent&& source)
      : FlutterKeyEvent(source),
        callback(std::move(source.callback)),
        user_data(source.user_data) {
    character = character_str.c_str();
  }

  FlutterKeyEventCallback callback;
  void* user_data;

 private:
  const std::string character_str;
};

class TestKeystate {
 public:
  void Set(int virtual_key, bool pressed, bool toggled_on = false) {
    state_[virtual_key] = (pressed ? kStateMaskPressed : 0) |
                          (toggled_on ? kStateMaskToggled : 0);
  }

  SHORT Get(int virtual_key) { return state_[virtual_key]; }

  KeyboardKeyEmbedderHandler::GetKeyStateHandler Getter() {
    return [this](int virtual_key) { return Get(virtual_key); };
  }

 private:
  std::map<int, SHORT> state_;
};

UINT DefaultMapVkToScan(UINT virtual_key, bool extended) {
  return MapVirtualKey(virtual_key,
                       extended ? MAPVK_VK_TO_VSC_EX : MAPVK_VK_TO_VSC);
}

constexpr uint64_t kScanCodeKeyA = 0x1e;
constexpr uint64_t kScanCodeAltLeft = 0x38;
constexpr uint64_t kScanCodeNumpad1 = 0x4f;
constexpr uint64_t kScanCodeNumLock = 0x45;
constexpr uint64_t kScanCodeControl = 0x1d;
constexpr uint64_t kScanCodeShiftLeft = 0x2a;
constexpr uint64_t kScanCodeShiftRight = 0x36;

constexpr uint64_t kVirtualKeyA = 0x41;

}  // namespace

using namespace ::flutter::testing::keycodes;

TEST(KeyboardKeyEmbedderHandlerTest, ConvertChar32ToUtf8) {
  std::string result;

  result = ConvertChar32ToUtf8(0x0024);
  EXPECT_EQ(result.length(), 1);
  EXPECT_EQ(result[0], '\x24');

  result = ConvertChar32ToUtf8(0x00A2);
  EXPECT_EQ(result.length(), 2);
  EXPECT_EQ(result[0], '\xC2');
  EXPECT_EQ(result[1], '\xA2');

  result = ConvertChar32ToUtf8(0x0939);
  EXPECT_EQ(result.length(), 3);
  EXPECT_EQ(result[0], '\xE0');
  EXPECT_EQ(result[1], '\xA4');
  EXPECT_EQ(result[2], '\xB9');

  result = ConvertChar32ToUtf8(0x10348);
  EXPECT_EQ(result.length(), 4);
  EXPECT_EQ(result[0], '\xF0');
  EXPECT_EQ(result[1], '\x90');
  EXPECT_EQ(result[2], '\x8D');
  EXPECT_EQ(result[3], '\x88');
}

// Test the most basic key events.
//
// Press, hold, and release key A on an US keyboard.
TEST(KeyboardKeyEmbedderHandlerTest, BasicKeyPressingAndHolding) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // Press KeyA.
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYDOWN, 'a', false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();
  key_state.Set(kVirtualKeyA, true);

  // Hold KeyA.
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYDOWN, 'a', false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, true);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeRepeat);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);

  event->callback(false, event->user_data);
  EXPECT_EQ(last_handled, false);
  results.clear();

  // Release KeyA.
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYUP, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);
  event->callback(false, event->user_data);
}

// Press numpad 1, toggle NumLock, and release numpad 1 on an US
// keyboard.
//
// This is special because the virtual key for numpad 1 will
// change in this process.
TEST(KeyboardKeyEmbedderHandlerTest, ToggleNumLockDuringNumpadPress) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // Press NumPad1.
  key_state.Set(VK_NUMPAD1, true);
  handler->KeyboardHook(
      VK_NUMPAD1, kScanCodeNumpad1, WM_KEYDOWN, 0, false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalNumpad1);
  EXPECT_EQ(event->logical, kLogicalNumpad1);
  // EXPECT_STREQ(event->character, "1"); // TODO
  EXPECT_EQ(event->synthesized, false);
  results.clear();

  // Press NumLock.
  key_state.Set(VK_NUMLOCK, true, true);
  handler->KeyboardHook(
      VK_NUMLOCK, kScanCodeNumLock, WM_KEYDOWN, 0, true, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);
  results.clear();

  // Release NumLock.
  key_state.Set(VK_NUMLOCK, false, true);
  handler->KeyboardHook(
      VK_NUMLOCK, kScanCodeNumLock, WM_KEYUP, 0, true, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);
  results.clear();

  // Release NumPad1. (The logical key is now NumpadEnd)
  handler->KeyboardHook(
      VK_END, kScanCodeNumpad1, WM_KEYUP, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalNumpad1);
  EXPECT_EQ(event->logical, kLogicalNumpad1);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);
  results.clear();
}

// Key presses that trigger IME should be ignored by this API (and handled by
// compose API).
TEST(KeyboardKeyEmbedderHandlerTest, ImeEventsAreIgnored) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // Press A in an IME
  last_handled = false;
  handler->KeyboardHook(
      VK_PROCESSKEY, kScanCodeKeyA, WM_KEYDOWN, 0, true, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, true);

  // The A key down should yield an empty event.
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->physical, 0);
  EXPECT_EQ(event->logical, 0);
  EXPECT_EQ(event->callback, nullptr);
  results.clear();

  // Release A in an IME
  last_handled = false;
  handler->KeyboardHook(
      // The up event for an IME press has a normal virtual key.
      kVirtualKeyA, kScanCodeKeyA, WM_KEYUP, 0, true, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, true);

  // The A key up should yield an empty event.
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->physical, 0);
  EXPECT_EQ(event->logical, 0);
  EXPECT_EQ(event->callback, nullptr);
  results.clear();

  // Press A out of an IME
  key_state.Set(kVirtualKeyA, true);
  last_handled = false;
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYDOWN, 0, true, false,
      [&last_handled](bool handled) { last_handled = handled; });
  // Not decided yet
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();

  last_handled = false;
  key_state.Set(kVirtualKeyA, false);
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYUP, 0, true, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
}

// Test if modifier keys that are told apart by the extended bit can be
// identified. (Their physical keys must be searched with the extended bit
// considered.)
TEST(KeyboardKeyEmbedderHandlerTest, ModifierKeysByExtendedBit) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // Press Ctrl left.
  last_handled = false;
  key_state.Set(VK_LCONTROL, true);
  handler->KeyboardHook(
      VK_LCONTROL, kScanCodeControl, WM_KEYDOWN, 0, false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalControlLeft);
  EXPECT_EQ(event->logical, kLogicalControlLeft);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();

  // Press Ctrl right.
  last_handled = false;
  key_state.Set(VK_RCONTROL, true);
  handler->KeyboardHook(
      VK_RCONTROL, kScanCodeControl, WM_KEYDOWN, 0, true, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalControlRight);
  EXPECT_EQ(event->logical, kLogicalControlRight);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();

  // Release Ctrl left.
  last_handled = false;
  key_state.Set(VK_LCONTROL, false);
  handler->KeyboardHook(
      VK_LCONTROL, kScanCodeControl, WM_KEYUP, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalControlLeft);
  EXPECT_EQ(event->logical, kLogicalControlLeft);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();

  // Release Ctrl right.
  last_handled = false;
  key_state.Set(VK_RCONTROL, false);
  handler->KeyboardHook(
      VK_RCONTROL, kScanCodeControl, WM_KEYUP, 0, true, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalControlRight);
  EXPECT_EQ(event->logical, kLogicalControlRight);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();
}

// Test if modifier keys that are told apart by the virtual key
// can be identified.
TEST(KeyboardKeyEmbedderHandlerTest, ModifierKeysByVirtualKey) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // Press Shift left.
  last_handled = false;
  key_state.Set(VK_LSHIFT, true);
  handler->KeyboardHook(
      VK_LSHIFT, kScanCodeShiftLeft, WM_KEYDOWN, 0, false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();

  // Press Shift right.
  last_handled = false;
  key_state.Set(VK_RSHIFT, true);
  handler->KeyboardHook(
      VK_RSHIFT, kScanCodeShiftRight, WM_KEYDOWN, 0, false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalShiftRight);
  EXPECT_EQ(event->logical, kLogicalShiftRight);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();

  // Release Shift left.
  last_handled = false;
  key_state.Set(VK_LSHIFT, false);
  handler->KeyboardHook(
      VK_LSHIFT, kScanCodeShiftLeft, WM_KEYUP, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();

  // Release Shift right.
  last_handled = false;
  key_state.Set(VK_RSHIFT, false);
  handler->KeyboardHook(
      VK_RSHIFT, kScanCodeShiftRight, WM_KEYUP, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalShiftRight);
  EXPECT_EQ(event->logical, kLogicalShiftRight);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();
}

// Test if modifiers left key down events are synthesized when left or right
// keys are not pressed.
TEST(KeyboardKeyEmbedderHandlerTest,
     SynthesizeModifierLeftKeyDownWhenNotPressed) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // Should synthesize shift left key down event.
  handler->SyncModifiersIfNeeded(kShift);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  results.clear();

  // Clear the pressing state.
  handler->SyncModifiersIfNeeded(0);
  results.clear();

  // Should synthesize control left key down event.
  handler->SyncModifiersIfNeeded(kControl);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalControlLeft);
  EXPECT_EQ(event->logical, kLogicalControlLeft);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
}

// Test if modifiers left key down events are not synthesized when left or right
// keys are pressed.
TEST(KeyboardKeyEmbedderHandlerTest, DoNotSynthesizeModifierDownWhenPressed) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // Should not synthesize shift left key down event when shift left key
  // is already pressed.
  handler->KeyboardHook(
      VK_LSHIFT, kScanCodeShiftLeft, WM_KEYDOWN, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  results.clear();
  handler->SyncModifiersIfNeeded(kShift);
  EXPECT_EQ(results.size(), 0);

  // Should not synthesize shift left key down event when shift right key
  // is already pressed.
  handler->KeyboardHook(
      VK_RSHIFT, kScanCodeShiftRight, WM_KEYDOWN, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  results.clear();
  handler->SyncModifiersIfNeeded(kShift);
  EXPECT_EQ(results.size(), 0);

  // Should not synthesize control left key down event when control left key
  // is already pressed.
  handler->KeyboardHook(
      VK_LCONTROL, kScanCodeControlLeft, WM_KEYDOWN, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  results.clear();
  handler->SyncModifiersIfNeeded(kControl);
  EXPECT_EQ(results.size(), 0);

  // Should not synthesize control left key down event when control right key
  // is already pressed .
  handler->KeyboardHook(
      VK_RCONTROL, kScanCodeControlRight, WM_KEYDOWN, 0, true, true,
      [&last_handled](bool handled) { last_handled = handled; });
  results.clear();
  handler->SyncModifiersIfNeeded(kControl);
  EXPECT_EQ(results.size(), 0);
}

// Test if modifiers keys up events are synthesized when left or right keys
// are pressed.
TEST(KeyboardKeyEmbedderHandlerTest, SynthesizeModifierUpWhenPressed) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // Should synthesize shift left key up event when shift left key is
  // already pressed and modifiers state is zero.
  handler->KeyboardHook(
      VK_LSHIFT, kScanCodeShiftLeft, WM_KEYDOWN, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  results.clear();
  handler->SyncModifiersIfNeeded(0);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  results.clear();

  // Should synthesize shift right key up event when shift right key is
  // already pressed and modifiers state is zero.
  handler->KeyboardHook(
      VK_RSHIFT, kScanCodeShiftRight, WM_KEYDOWN, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  results.clear();
  handler->SyncModifiersIfNeeded(0);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalShiftRight);
  EXPECT_EQ(event->logical, kLogicalShiftRight);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  results.clear();

  // Should synthesize control left key up event when control left key is
  // already pressed and modifiers state is zero.
  handler->KeyboardHook(
      VK_LCONTROL, kScanCodeControlLeft, WM_KEYDOWN, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  results.clear();
  handler->SyncModifiersIfNeeded(0);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalControlLeft);
  EXPECT_EQ(event->logical, kLogicalControlLeft);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  results.clear();

  // Should synthesize control right key up event when control right key is
  // already pressed and modifiers state is zero.
  handler->KeyboardHook(
      VK_RCONTROL, kScanCodeControlRight, WM_KEYDOWN, 0, true, true,
      [&last_handled](bool handled) { last_handled = handled; });
  results.clear();
  handler->SyncModifiersIfNeeded(0);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalControlRight);
  EXPECT_EQ(event->logical, kLogicalControlRight);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  results.clear();
}

// Test if modifiers key up events are not synthesized when left or right
// keys are not pressed.
TEST(KeyboardKeyEmbedderHandlerTest, DoNotSynthesizeModifierUpWhenNotPressed) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // Should not synthesize up events when no modifier key is pressed
  // in pressing state and modifiers state is zero.
  handler->SyncModifiersIfNeeded(0);
  EXPECT_EQ(results.size(), 0);
}

TEST(KeyboardKeyEmbedderHandlerTest, RepeatedDownIsIgnored) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);
  last_handled = false;

  // Press A (should yield a normal event)
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYDOWN, 'a', false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();

  // KeyA's key up is missed.

  // Press A again (should yield an empty event)
  last_handled = false;
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYDOWN, 'a', false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, true);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->physical, 0);
  EXPECT_EQ(event->logical, 0);
  EXPECT_EQ(event->callback, nullptr);
  results.clear();
}

TEST(KeyboardKeyEmbedderHandlerTest, AbruptRepeatIsConvertedToDown) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);
  last_handled = false;

  key_state.Set(kVirtualKeyA, true);

  // Press A (with was_down true)
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYDOWN, 'a', false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();

  // Release A
  last_handled = false;
  key_state.Set(kVirtualKeyA, false);
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYUP, 'a', false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();
}

TEST(KeyboardKeyEmbedderHandlerTest, AbruptUpIsIgnored) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);
  last_handled = false;

  // KeyA's key down is missed.

  key_state.Set(kVirtualKeyA, true);

  // Press A again (should yield an empty event)
  last_handled = false;
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYUP, 'a', false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, true);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->physical, 0);
  EXPECT_EQ(event->logical, 0);
  EXPECT_EQ(event->callback, nullptr);
  results.clear();
}

TEST(KeyboardKeyEmbedderHandlerTest, SynthesizeForDesyncPressingState) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // A key down of control left is missed.
  key_state.Set(VK_LCONTROL, true);

  // Send a normal event
  key_state.Set(kVirtualKeyA, true);
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYDOWN, 'a', false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 2);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalControlLeft);
  EXPECT_EQ(event->logical, kLogicalControlLeft);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  EXPECT_EQ(event->callback, nullptr);

  event = &results[1];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);

  last_handled = true;
  event->callback(false, event->user_data);
  EXPECT_EQ(last_handled, false);
  results.clear();

  // A key down of control right is missed.
  key_state.Set(VK_LCONTROL, false);

  // Hold KeyA.
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYDOWN, 'a', false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 2);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalControlLeft);
  EXPECT_EQ(event->logical, kLogicalControlLeft);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  EXPECT_EQ(event->callback, nullptr);

  event = &results[1];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeRepeat);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);

  last_handled = true;
  event->callback(false, event->user_data);
  EXPECT_EQ(last_handled, false);
  results.clear();

  // Release KeyA.
  key_state.Set(kVirtualKeyA, false);
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYUP, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);
  event->callback(false, event->user_data);
}

TEST(KeyboardKeyEmbedderHandlerTest, SynthesizeForDesyncToggledState) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // The NumLock is desynchronized by toggled on
  key_state.Set(VK_NUMLOCK, false, true);

  // Send a normal event
  key_state.Set(kVirtualKeyA, true);
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYDOWN, 'a', false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 3);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);

  event = &results[1];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);

  event = &results[2];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();

  // Test if the NumLock is mis-toggled while it should also be pressed
  key_state.Set(VK_NUMLOCK, true, true);

  // Send a normal event
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYDOWN, 'a', false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, true);
  EXPECT_EQ(results.size(), 2);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);

  event = &results[1];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeRepeat);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);

  event->callback(false, event->user_data);
  EXPECT_EQ(last_handled, false);
  results.clear();

  // Numlock is pressed at this moment.

  // Test if the NumLock is mis-toggled while it should also be released
  key_state.Set(VK_NUMLOCK, false, false);

  // Send a normal event
  key_state.Set(kVirtualKeyA, false);
  handler->KeyboardHook(
      kVirtualKeyA, kScanCodeKeyA, WM_KEYUP, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(results.size(), 4);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  EXPECT_EQ(event->callback, nullptr);

  event = &results[1];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  EXPECT_EQ(event->callback, nullptr);

  event = &results[2];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  EXPECT_EQ(event->callback, nullptr);

  event = &results[3];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);
  event->callback(false, event->user_data);
}

TEST(KeyboardKeyEmbedderHandlerTest,
     SynthesizeForDesyncToggledStateByItselfsUp) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // When NumLock is down
  key_state.Set(VK_NUMLOCK, true, true);
  handler->KeyboardHook(
      VK_NUMLOCK, kScanCodeNumLock, WM_KEYDOWN, 0, true, false,
      [&last_handled](bool handled) { last_handled = handled; });
  event = &results.back();
  event->callback(false, event->user_data);
  results.clear();

  // Numlock is desynchronized by being off and released
  key_state.Set(VK_NUMLOCK, false, false);
  // Send a NumLock key up
  handler->KeyboardHook(
      VK_NUMLOCK, kScanCodeNumLock, WM_KEYUP, 0, true, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 3);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  EXPECT_EQ(event->callback, nullptr);

  event = &results[1];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  EXPECT_EQ(event->callback, nullptr);

  event = &results[2];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);

  last_handled = false;
  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
}

TEST(KeyboardKeyEmbedderHandlerTest,
     SynthesizeForDesyncToggledStateByItselfsDown) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  // NumLock is started up and disabled
  key_state.Set(VK_NUMLOCK, false, false);
  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // NumLock is toggled somewhere else
  // key_state.Set(VK_NUMLOCK, false, true);

  // NumLock is pressed
  key_state.Set(VK_NUMLOCK, true, false);
  handler->KeyboardHook(
      VK_NUMLOCK, kScanCodeNumLock, WM_KEYDOWN, 0, true, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  // 4 total events should be fired:
  // Pre-synchronization toggle, pre-sync press,
  // main event, and post-sync press.
  EXPECT_EQ(results.size(), 4);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  EXPECT_EQ(event->callback, nullptr);

  event = &results[1];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, true);
  EXPECT_EQ(event->callback, nullptr);

  event = &results[2];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);

  last_handled = false;
  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
}

TEST(KeyboardKeyEmbedderHandlerTest, SynthesizeWithInitialTogglingState) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  // The app starts with NumLock toggled on
  key_state.Set(VK_NUMLOCK, false, true);

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // NumLock key down
  key_state.Set(VK_NUMLOCK, true, false);
  handler->KeyboardHook(
      VK_NUMLOCK, kScanCodeNumLock, WM_KEYDOWN, 0, true, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = &results[0];
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalNumLock);
  EXPECT_EQ(event->logical, kLogicalNumLock);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();
}

TEST(KeyboardKeyEmbedderHandlerTest, SysKeyPress) {
  TestKeystate key_state;
  std::vector<TestFlutterKeyEvent> results;
  TestFlutterKeyEvent* event;
  bool last_handled = false;

  std::unique_ptr<KeyboardKeyEmbedderHandler> handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [&results](const FlutterKeyEvent& event,
                     FlutterKeyEventCallback callback, void* user_data) {
            results.emplace_back(event, callback, user_data);
          },
          key_state.Getter(), DefaultMapVkToScan);

  // Press KeyAltLeft.
  key_state.Set(VK_LMENU, true);
  handler->KeyboardHook(
      VK_LMENU, kScanCodeAltLeft, WM_SYSKEYDOWN, 0, false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(results.size(), 1);
  event = results.data();
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalAltLeft);
  EXPECT_EQ(event->logical, kLogicalAltLeft);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);

  event->callback(true, event->user_data);
  EXPECT_EQ(last_handled, true);
  results.clear();

  // Release KeyAltLeft.
  key_state.Set(VK_LMENU, false);
  handler->KeyboardHook(
      VK_LMENU, kScanCodeAltLeft, WM_SYSKEYUP, 0, false, true,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(results.size(), 1);
  event = results.data();
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalAltLeft);
  EXPECT_EQ(event->logical, kLogicalAltLeft);
  EXPECT_STREQ(event->character, "");
  EXPECT_EQ(event->synthesized, false);
  event->callback(false, event->user_data);
}

}  // namespace testing
}  // namespace flutter
