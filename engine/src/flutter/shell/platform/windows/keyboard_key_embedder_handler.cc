// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/keyboard_key_embedder_handler.h"

#include <windows.h>

#include <chrono>
#include <string>

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/windows/keyboard_utils.h"

namespace flutter {

namespace {
// An arbitrary size for the character cache in bytes.
//
// It should hold a UTF-32 character encoded in UTF-8 as well as the trailing
// '\0'.
constexpr size_t kCharacterCacheSize = 8;

constexpr SHORT kStateMaskToggled = 0x01;
constexpr SHORT kStateMaskPressed = 0x80;

const char* empty_character = "";

// Get some bits of the char, from the start'th bit from the right (excluded)
// to the end'th bit from the right (included).
//
// For example, _GetBit(0x1234, 8, 4) => 0x3.
char _GetBit(char32_t ch, size_t start, size_t end) {
  return (ch >> end) & ((1 << (start - end)) - 1);
}
}  // namespace

std::string ConvertChar32ToUtf8(char32_t ch) {
  std::string result;
  FML_DCHECK(0 <= ch && ch <= 0x10FFFF) << "Character out of range";
  if (ch <= 0x007F) {
    result.push_back(ch);
  } else if (ch <= 0x07FF) {
    result.push_back(0b11000000 + _GetBit(ch, 11, 6));
    result.push_back(0b10000000 + _GetBit(ch, 6, 0));
  } else if (ch <= 0xFFFF) {
    result.push_back(0b11100000 + _GetBit(ch, 16, 12));
    result.push_back(0b10000000 + _GetBit(ch, 12, 6));
    result.push_back(0b10000000 + _GetBit(ch, 6, 0));
  } else {
    result.push_back(0b11110000 + _GetBit(ch, 21, 18));
    result.push_back(0b10000000 + _GetBit(ch, 18, 12));
    result.push_back(0b10000000 + _GetBit(ch, 12, 6));
    result.push_back(0b10000000 + _GetBit(ch, 6, 0));
  }
  return result;
}

KeyboardKeyEmbedderHandler::KeyboardKeyEmbedderHandler(
    SendEventHandler send_event,
    GetKeyStateHandler get_key_state,
    MapVirtualKeyToScanCode map_virtual_key_to_scan_code)
    : perform_send_event_(send_event),
      get_key_state_(get_key_state),
      response_id_(1) {
  InitCriticalKeys(map_virtual_key_to_scan_code);
}

KeyboardKeyEmbedderHandler::~KeyboardKeyEmbedderHandler() = default;

static bool isEasciiPrintable(int codeUnit) {
  return (codeUnit <= 0x7f && codeUnit >= 0x20) ||
         (codeUnit <= 0xff && codeUnit >= 0x80);
}

// Converts upper letters to lower letters in ASCII and extended ASCII, and
// returns as-is otherwise.
//
// Independent of locale.
static uint64_t toLower(uint64_t n) {
  constexpr uint64_t lower_a = 0x61;
  constexpr uint64_t upper_a = 0x41;
  constexpr uint64_t upper_z = 0x5a;

  constexpr uint64_t lower_a_grave = 0xe0;
  constexpr uint64_t upper_a_grave = 0xc0;
  constexpr uint64_t upper_thorn = 0xde;
  constexpr uint64_t division = 0xf7;

  // ASCII range.
  if (n >= upper_a && n <= upper_z) {
    return n - upper_a + lower_a;
  }

  // EASCII range.
  if (n >= upper_a_grave && n <= upper_thorn && n != division) {
    return n - upper_a_grave + lower_a_grave;
  }

  return n;
}

// Transform scancodes sent by windows to scancodes written in Chromium spec.
static uint16_t normalizeScancode(int windowsScanCode, bool extended) {
  // In Chromium spec the extended bit is shown as 0xe000 bit,
  // e.g. PageUp is represented as 0xe049.
  return (windowsScanCode & 0xff) | (extended ? 0xe000 : 0);
}

uint64_t KeyboardKeyEmbedderHandler::ApplyPlaneToId(uint64_t id,
                                                    uint64_t plane) {
  return (id & valueMask) | plane;
}

uint64_t KeyboardKeyEmbedderHandler::GetPhysicalKey(int scancode,
                                                    bool extended) {
  int chromiumScancode = normalizeScancode(scancode, extended);
  auto resultIt = windowsToPhysicalMap_.find(chromiumScancode);
  if (resultIt != windowsToPhysicalMap_.end())
    return resultIt->second;
  return ApplyPlaneToId(scancode, windowsPlane);
}

uint64_t KeyboardKeyEmbedderHandler::GetLogicalKey(int key,
                                                   bool extended,
                                                   int scancode) {
  if (key == VK_PROCESSKEY) {
    return VK_PROCESSKEY;
  }

  // Normally logical keys should only be derived from key codes, but since some
  // key codes are either 0 or ambiguous (multiple keys using the same key
  // code), these keys are resolved by scan codes.
  auto numpadIter =
      scanCodeToLogicalMap_.find(normalizeScancode(scancode, extended));
  if (numpadIter != scanCodeToLogicalMap_.cend())
    return numpadIter->second;

  // Check if the keyCode is one we know about and have a mapping for.
  auto logicalIt = windowsToLogicalMap_.find(key);
  if (logicalIt != windowsToLogicalMap_.cend())
    return logicalIt->second;

  // Upper case letters should be normalized into lower case letters.
  if (isEasciiPrintable(key)) {
    return ApplyPlaneToId(toLower(key), unicodePlane);
  }

  return ApplyPlaneToId(toLower(key), windowsPlane);
}

void KeyboardKeyEmbedderHandler::KeyboardHookImpl(
    int key,
    int scancode,
    int action,
    char32_t character,
    bool extended,
    bool was_down,
    std::function<void(bool)> callback) {
  const uint64_t physical_key = GetPhysicalKey(scancode, extended);
  const uint64_t logical_key = GetLogicalKey(key, extended, scancode);
  FML_DCHECK(action == WM_KEYDOWN || action == WM_KEYUP ||
             action == WM_SYSKEYDOWN || action == WM_SYSKEYUP);

  auto last_logical_record_iter = pressingRecords_.find(physical_key);
  bool had_record = last_logical_record_iter != pressingRecords_.end();
  uint64_t last_logical_record =
      had_record ? last_logical_record_iter->second : 0;

  // The logical key for the current "tap sequence".
  //
  // Key events are formed in tap sequences: down, repeats, up. The logical key
  // stays consistent throughout a tap sequence, which is this value.
  uint64_t sequence_logical_key =
      had_record ? last_logical_record : logical_key;

  if (sequence_logical_key == VK_PROCESSKEY) {
    // VK_PROCESSKEY means that the key press is used by an IME. These key
    // presses are considered handled and not sent to Flutter. These events must
    // be filtered by result_logical_key because the key up event of such
    // presses uses the "original" logical key.
    callback(true);
    return;
  }

  const bool is_event_down = action == WM_KEYDOWN || action == WM_SYSKEYDOWN;

  bool event_key_can_be_repeat = true;
  UpdateLastSeenCriticalKey(key, physical_key, sequence_logical_key);
  // Synchronize the toggled states of critical keys (such as whether CapsLocks
  // is enabled). Toggled states can only be changed upon a down event, so if
  // the recorded toggled state does not match the true state, this function
  // will synthesize (an up event if the key is recorded pressed, then) a down
  // event.
  //
  // After this function, all critical keys will have their toggled state
  // updated to the true state, while the critical keys whose toggled state have
  // been changed will be pressed regardless of their true pressed state.
  // Updating the pressed state will be done by
  // SynchronizeCriticalPressedStates.
  SynchronizeCriticalToggledStates(key, is_event_down,
                                   &event_key_can_be_repeat);
  // Synchronize the pressed states of critical keys (such as whether CapsLocks
  // is pressed).
  //
  // After this function, all critical keys except for the target key will have
  // their toggled state and pressed state matched with their true states. The
  // target key's pressed state will be updated immediately after this.
  SynchronizeCriticalPressedStates(key, physical_key, is_event_down,
                                   event_key_can_be_repeat);

  // Reassess the last logical record in case pressingRecords_ was modified
  // by the above synchronization methods.
  last_logical_record_iter = pressingRecords_.find(physical_key);
  had_record = last_logical_record_iter != pressingRecords_.end();
  last_logical_record = had_record ? last_logical_record_iter->second : 0;

  // The resulting event's `type`.
  FlutterKeyEventType type;
  character = UndeadChar(character);
  char character_bytes[kCharacterCacheSize];
  // What pressingRecords_[physical_key] should be after the KeyboardHookImpl
  // returns (0 if the entry should be removed).
  uint64_t eventual_logical_record;
  uint64_t result_logical_key;

  if (is_event_down) {
    if (had_record) {
      if (was_down) {
        // A normal repeated key.
        type = kFlutterKeyEventTypeRepeat;
        FML_DCHECK(had_record);
        ConvertUtf32ToUtf8_(character_bytes, character);
        eventual_logical_record = last_logical_record;
        result_logical_key = last_logical_record;
      } else {
        // A non-repeated key has been pressed that has the exact physical key
        // as a currently pressed one, usually indicating multiple keyboards are
        // pressing keys with the same physical key, or the up event was lost
        // during a loss of focus. The down event is ignored.
        callback(true);
        return;
      }
    } else {
      // A normal down event (whether the system event is a repeat or not).
      type = kFlutterKeyEventTypeDown;
      FML_DCHECK(!had_record);
      ConvertUtf32ToUtf8_(character_bytes, character);
      eventual_logical_record = logical_key;
      result_logical_key = logical_key;
    }
  } else {  // isPhysicalDown is false
    if (last_logical_record == 0) {
      // The physical key has been released before. It might indicate a missed
      // event due to loss of focus, or multiple keyboards pressed keys with the
      // same physical key. Ignore the up event.
      callback(true);
      return;
    } else {
      // A normal up event.
      type = kFlutterKeyEventTypeUp;
      FML_DCHECK(had_record);
      // Up events never have character.
      character_bytes[0] = '\0';
      eventual_logical_record = 0;
      result_logical_key = last_logical_record;
    }
  }

  if (eventual_logical_record != 0) {
    pressingRecords_[physical_key] = eventual_logical_record;
  } else {
    auto record_iter = pressingRecords_.find(physical_key);
    // Assert this in debug mode. But in cases that it doesn't satisfy
    // (such as due to a bug), make sure the `erase` is only called
    // with a valid value to avoid crashing.
    if (record_iter != pressingRecords_.end()) {
      pressingRecords_.erase(record_iter);
    } else {
      FML_DCHECK(false);
    }
  }

  FlutterKeyEvent key_data{
      .struct_size = sizeof(FlutterKeyEvent),
      .timestamp = static_cast<double>(
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::high_resolution_clock::now().time_since_epoch())
              .count()),
      .type = type,
      .physical = physical_key,
      .logical = result_logical_key,
      .character = character_bytes,
      .synthesized = false,
  };

  response_id_ += 1;
  uint64_t response_id = response_id_;
  PendingResponse pending{
      .callback =
          [this, callback = std::move(callback)](bool handled,
                                                 uint64_t response_id) {
            auto found = pending_responses_.find(response_id);
            if (found != pending_responses_.end()) {
              pending_responses_.erase(found);
            }
            callback(handled);
          },
      .response_id = response_id,
  };
  auto pending_ptr = std::make_unique<PendingResponse>(std::move(pending));
  pending_responses_[response_id] = std::move(pending_ptr);
  SendEvent(key_data, KeyboardKeyEmbedderHandler::HandleResponse,
            reinterpret_cast<void*>(pending_responses_[response_id].get()));

  // Post-event synchronization. It is useful in cases where the true pressing
  // state does not match the event type. For example, a CapsLock down event is
  // received despite that GetKeyState says that CapsLock is not pressed. In
  // such case, post-event synchronization will synthesize a CapsLock up event
  // after the main event.
  SynchronizeCriticalPressedStates(key, physical_key, is_event_down,
                                   event_key_can_be_repeat);
}

void KeyboardKeyEmbedderHandler::KeyboardHook(
    int key,
    int scancode,
    int action,
    char32_t character,
    bool extended,
    bool was_down,
    std::function<void(bool)> callback) {
  sent_any_events = false;
  KeyboardHookImpl(key, scancode, action, character, extended, was_down,
                   std::move(callback));
  if (!sent_any_events) {
    FlutterKeyEvent empty_event{
        .struct_size = sizeof(FlutterKeyEvent),
        .timestamp = static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count()),
        .type = kFlutterKeyEventTypeDown,
        .physical = 0,
        .logical = 0,
        .character = empty_character,
        .synthesized = false,
    };
    SendEvent(empty_event, nullptr, nullptr);
  }
}

std::map<uint64_t, uint64_t> KeyboardKeyEmbedderHandler::GetPressedState() {
  return pressingRecords_;
}

void KeyboardKeyEmbedderHandler::UpdateLastSeenCriticalKey(
    int virtual_key,
    uint64_t physical_key,
    uint64_t logical_key) {
  auto found = critical_keys_.find(virtual_key);
  if (found != critical_keys_.end()) {
    found->second.physical_key = physical_key;
    found->second.logical_key = logical_key;
  }
}

void KeyboardKeyEmbedderHandler::SynchronizeCriticalToggledStates(
    int event_virtual_key,
    bool is_event_down,
    bool* event_key_can_be_repeat) {
  //    NowState ---------------->  PreEventState --------------> TrueState
  //              Synchronization                      Event
  for (auto& kv : critical_keys_) {
    UINT virtual_key = kv.first;
    CriticalKey& key_info = kv.second;
    if (key_info.physical_key == 0) {
      // Never seen this key.
      continue;
    }
    FML_DCHECK(key_info.logical_key != 0);

    // Check toggling state first, because it might alter pressing state.
    if (key_info.check_toggled) {
      const bool target_is_pressed =
          pressingRecords_.find(key_info.physical_key) !=
          pressingRecords_.end();
      // The togglable keys observe a 4-phase cycle:
      //
      //  Phase#   0          1          2         3
      //   Event       Down        Up        Down      Up
      // Pressed   0          1          0         1
      // Toggled   0          1          1         0
      const bool true_toggled = get_key_state_(virtual_key) & kStateMaskToggled;
      bool pre_event_toggled = true_toggled;
      // Check if the main event's key is the key being checked. If it's the
      // non-repeat down event, toggle the state.
      if (virtual_key == event_virtual_key && !target_is_pressed &&
          is_event_down) {
        pre_event_toggled = !pre_event_toggled;
      }
      if (key_info.toggled_on != pre_event_toggled) {
        // If the key is pressed, release it first.
        if (target_is_pressed) {
          SendEvent(SynthesizeSimpleEvent(
                        kFlutterKeyEventTypeUp, key_info.physical_key,
                        key_info.logical_key, empty_character),
                    nullptr, nullptr);
        }
        // Synchronizing toggle state always ends with the key being pressed.
        pressingRecords_[key_info.physical_key] = key_info.logical_key;
        SendEvent(SynthesizeSimpleEvent(kFlutterKeyEventTypeDown,
                                        key_info.physical_key,
                                        key_info.logical_key, empty_character),
                  nullptr, nullptr);
        *event_key_can_be_repeat = false;
      }
      key_info.toggled_on = true_toggled;
    }
  }
}

void KeyboardKeyEmbedderHandler::SynchronizeCriticalPressedStates(
    int event_virtual_key,
    int event_physical_key,
    bool is_event_down,
    bool event_key_can_be_repeat) {
  // During an incoming event, there might be a synthesized Flutter event for
  // each key of each pressing goal, followed by an eventual main Flutter
  // event.
  //
  //    NowState ---------------->  PreEventState --------------> TrueState
  //              Synchronization                      Event
  //
  // The goal of the synchronization algorithm is to derive a pre-event state
  // that can satisfy the true state (`true_pressed`) after the event, and that
  // requires as few synthesized events based on the current state
  // (`now_pressed`) as possible.
  for (auto& kv : critical_keys_) {
    UINT virtual_key = kv.first;
    CriticalKey& key_info = kv.second;
    if (key_info.physical_key == 0) {
      // Never seen this key.
      continue;
    }
    FML_DCHECK(key_info.logical_key != 0);
    if (key_info.check_pressed) {
      SHORT true_pressed = get_key_state_(virtual_key) & kStateMaskPressed;
      auto pressing_record_iter = pressingRecords_.find(key_info.physical_key);
      bool now_pressed = pressing_record_iter != pressingRecords_.end();
      bool pre_event_pressed = true_pressed;
      // Check if the main event is the key being checked to get the correct
      // target state.
      if (is_event_down) {
        // For down events, this key is the event key if they have the same
        // virtual key, because virtual key represents "functionality."
        //
        // In that case, normally Flutter should synthesize nothing since the
        // resulting event can adapt to the current state by dispatching either
        // a down or a repeat event. However, in certain cases (when Flutter has
        // just synchronized the key's toggling state) the event must not be a
        // repeat event.
        if (virtual_key == event_virtual_key) {
          if (event_key_can_be_repeat) {
            continue;
          } else {
            pre_event_pressed = false;
          }
        }
      } else {
        // For up events, this key is the event key if they have the same
        // physical key, because it is necessary to ensure that the physical
        // key is correctly released.
        //
        // In that case, although the previous state should be pressed, don't
        // synthesize a down event even if it's not. The later code will handle
        // such cases by skipping abrupt up events. Obviously don't synthesize
        // up events either.
        if (event_physical_key == key_info.physical_key) {
          continue;
        }
      }
      if (now_pressed != pre_event_pressed) {
        if (now_pressed) {
          pressingRecords_.erase(pressing_record_iter);
        } else {
          pressingRecords_[key_info.physical_key] = key_info.logical_key;
        }
        const char* empty_character = "";
        SendEvent(
            SynthesizeSimpleEvent(
                now_pressed ? kFlutterKeyEventTypeUp : kFlutterKeyEventTypeDown,
                key_info.physical_key, key_info.logical_key, empty_character),
            nullptr, nullptr);
      }
    }
  }
}

void KeyboardKeyEmbedderHandler::SyncModifiersIfNeeded(int modifiers_state) {
  // TODO(bleroux): consider exposing these constants in flutter_key_map.g.cc?
  const uint64_t physical_shift_left =
      windowsToPhysicalMap_.at(kScanCodeShiftLeft);
  const uint64_t physical_shift_right =
      windowsToPhysicalMap_.at(kScanCodeShiftRight);
  const uint64_t logical_shift_left =
      windowsToLogicalMap_.at(kKeyCodeShiftLeft);
  const uint64_t physical_control_left =
      windowsToPhysicalMap_.at(kScanCodeControlLeft);
  const uint64_t physical_control_right =
      windowsToPhysicalMap_.at(kScanCodeControlRight);
  const uint64_t logical_control_left =
      windowsToLogicalMap_.at(kKeyCodeControlLeft);

  bool shift_pressed = (modifiers_state & kShift) != 0;
  SynthesizeIfNeeded(physical_shift_left, physical_shift_right,
                     logical_shift_left, shift_pressed);
  bool control_pressed = (modifiers_state & kControl) != 0;
  SynthesizeIfNeeded(physical_control_left, physical_control_right,
                     logical_control_left, control_pressed);
}

void KeyboardKeyEmbedderHandler::SynthesizeIfNeeded(uint64_t physical_left,
                                                    uint64_t physical_right,
                                                    uint64_t logical_left,
                                                    bool is_pressed) {
  auto pressing_record_iter_left = pressingRecords_.find(physical_left);
  bool left_pressed = pressing_record_iter_left != pressingRecords_.end();
  auto pressing_record_iter_right = pressingRecords_.find(physical_right);
  bool right_pressed = pressing_record_iter_right != pressingRecords_.end();
  bool already_pressed = left_pressed || right_pressed;
  bool synthesize_down = is_pressed && !already_pressed;
  bool synthesize_up = !is_pressed && already_pressed;

  if (synthesize_down) {
    SendSynthesizeDownEvent(physical_left, logical_left);
  }

  if (synthesize_up && left_pressed) {
    uint64_t known_logical = pressing_record_iter_left->second;
    SendSynthesizeUpEvent(physical_left, known_logical);
  }

  if (synthesize_up && right_pressed) {
    uint64_t known_logical = pressing_record_iter_right->second;
    SendSynthesizeUpEvent(physical_right, known_logical);
  }
}

void KeyboardKeyEmbedderHandler::SendSynthesizeDownEvent(uint64_t physical,
                                                         uint64_t logical) {
  SendEvent(
      SynthesizeSimpleEvent(kFlutterKeyEventTypeDown, physical, logical, ""),
      nullptr, nullptr);
  pressingRecords_[physical] = logical;
};

void KeyboardKeyEmbedderHandler::SendSynthesizeUpEvent(uint64_t physical,
                                                       uint64_t logical) {
  SendEvent(
      SynthesizeSimpleEvent(kFlutterKeyEventTypeUp, physical, logical, ""),
      nullptr, nullptr);
  pressingRecords_.erase(physical);
};

void KeyboardKeyEmbedderHandler::HandleResponse(bool handled, void* user_data) {
  PendingResponse* pending = reinterpret_cast<PendingResponse*>(user_data);
  auto callback = std::move(pending->callback);
  callback(handled, pending->response_id);
}

void KeyboardKeyEmbedderHandler::InitCriticalKeys(
    MapVirtualKeyToScanCode map_virtual_key_to_scan_code) {
  auto createCheckedKey = [this, &map_virtual_key_to_scan_code](
                              UINT virtual_key, bool extended,
                              bool check_pressed,
                              bool check_toggled) -> CriticalKey {
    UINT scan_code = map_virtual_key_to_scan_code(virtual_key, extended);
    return CriticalKey{
        .physical_key = GetPhysicalKey(scan_code, extended),
        .logical_key = GetLogicalKey(virtual_key, extended, scan_code),
        .check_pressed = check_pressed || check_toggled,
        .check_toggled = check_toggled,
        .toggled_on = check_toggled
                          ? !!(get_key_state_(virtual_key) & kStateMaskToggled)
                          : false,
    };
  };

  critical_keys_.emplace(VK_LSHIFT,
                         createCheckedKey(VK_LSHIFT, false, true, false));
  critical_keys_.emplace(VK_RSHIFT,
                         createCheckedKey(VK_RSHIFT, false, true, false));
  critical_keys_.emplace(VK_LCONTROL,
                         createCheckedKey(VK_LCONTROL, false, true, false));
  critical_keys_.emplace(VK_RCONTROL,
                         createCheckedKey(VK_RCONTROL, true, true, false));
  critical_keys_.emplace(VK_LMENU,
                         createCheckedKey(VK_LMENU, false, true, false));
  critical_keys_.emplace(VK_RMENU,
                         createCheckedKey(VK_RMENU, true, true, false));
  critical_keys_.emplace(VK_LWIN, createCheckedKey(VK_LWIN, true, true, false));
  critical_keys_.emplace(VK_RWIN, createCheckedKey(VK_RWIN, true, true, false));
  critical_keys_.emplace(VK_CAPITAL,
                         createCheckedKey(VK_CAPITAL, false, true, true));
  critical_keys_.emplace(VK_SCROLL,
                         createCheckedKey(VK_SCROLL, false, true, true));
  critical_keys_.emplace(VK_NUMLOCK,
                         createCheckedKey(VK_NUMLOCK, true, true, true));
}

void KeyboardKeyEmbedderHandler::ConvertUtf32ToUtf8_(char* out, char32_t ch) {
  if (ch == 0) {
    out[0] = '\0';
    return;
  }
  std::string result = ConvertChar32ToUtf8(ch);
  strcpy_s(out, kCharacterCacheSize, result.c_str());
}

FlutterKeyEvent KeyboardKeyEmbedderHandler::SynthesizeSimpleEvent(
    FlutterKeyEventType type,
    uint64_t physical,
    uint64_t logical,
    const char* character) {
  return FlutterKeyEvent{
      .struct_size = sizeof(FlutterKeyEvent),
      .timestamp = static_cast<double>(
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::high_resolution_clock::now().time_since_epoch())
              .count()),
      .type = type,
      .physical = physical,
      .logical = logical,
      .character = character,
      .synthesized = true,
  };
}

void KeyboardKeyEmbedderHandler::SendEvent(const FlutterKeyEvent& event,
                                           FlutterKeyEventCallback callback,
                                           void* user_data) {
  sent_any_events = true;
  perform_send_event_(event, callback, user_data);
}

}  // namespace flutter
