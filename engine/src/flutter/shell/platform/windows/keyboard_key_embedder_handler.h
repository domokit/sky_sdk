// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_KEYBOARD_KEY_EMBEDDER_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_KEYBOARD_KEY_EMBEDDER_HANDLER_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/windows/keyboard_key_handler.h"

namespace flutter {

// Encode a 32-bit unicode code point into a UTF-8 byte array.
//
// See https://en.wikipedia.org/wiki/UTF-8#Encoding for the algorithm.
std::string ConvertChar32ToUtf8(char32_t ch);

// A delegate of |KeyboardKeyHandler| that handles events by sending
// converted |FlutterKeyEvent|s through the embedder API.
//
// This class communicates with the HardwareKeyboard API in the framework.
//
// Every key event must result in at least one FlutterKeyEvent, even an empty
// one (both logical and physical IDs are 0). This ensures that raw key
// messages are always preceded by key data so that the transit mode is
// correctly inferred. (Technically only the first key event needs so, but for
// simplicity.)
class KeyboardKeyEmbedderHandler
    : public KeyboardKeyHandler::KeyboardKeyHandlerDelegate {
 public:
  using SendEventHandler =
      std::function<void(const FlutterKeyEvent& /* event */,
                         FlutterKeyEventCallback /* callback */,
                         void* /* user_data */)>;
  using GetKeyStateHandler = std::function<SHORT(int /* nVirtKey */)>;
  using MapVirtualKeyToScanCode =
      std::function<SHORT(UINT /* nVirtKey */, bool /* extended */)>;

  // Build a KeyboardKeyEmbedderHandler.
  //
  // Use `send_event` to define how the class should dispatch converted
  // flutter events, as well as how to receive the response, to the engine. It's
  // typically FlutterWindowsEngine::SendKeyEvent. The 2nd and 3rd parameter
  // of the SendEventHandler call might be nullptr.
  //
  // Use `get_key_state` to define how the class should get a reliable result of
  // the state for a virtual key. It's typically Win32's GetKeyState, but can
  // also be nullptr (for UWP).
  //
  // Use `map_vk_to_scan` to define how the class should get map a virtual key
  // to a scan code. It's typically Win32's MapVirtualKey, but can also be
  // nullptr (for UWP).
  explicit KeyboardKeyEmbedderHandler(SendEventHandler send_event,
                                      GetKeyStateHandler get_key_state,
                                      MapVirtualKeyToScanCode map_vk_to_scan);

  virtual ~KeyboardKeyEmbedderHandler();

  // |KeyboardHandlerBase|
  void KeyboardHook(int key,
                    int scancode,
                    int action,
                    char32_t character,
                    bool extended,
                    bool was_down,
                    std::function<void(bool)> callback) override;

  void SyncModifiersIfNeeded(int modifiers_state) override;

  std::map<uint64_t, uint64_t> GetPressedState() override;

 private:
  struct PendingResponse {
    std::function<void(bool, uint64_t)> callback;
    uint64_t response_id;
  };

  // The information for a virtual key that's important enough that its
  // state is checked after every event.
  struct CriticalKey {
    // Last seen value of physical key and logical key for the virtual key.
    //
    // Used to synthesize down events.
    uint64_t physical_key;
    uint64_t logical_key;

    // Whether to ensure the pressing state of the key (usually for modifier
    // keys).
    bool check_pressed;
    // Whether to ensure the toggled state of the key (usually for lock keys).
    //
    // If this is true, `check_pressed` must be true.
    bool check_toggled;
    // Whether the lock key is currently toggled on.
    bool toggled_on;
  };

  // Implements the core logic of |KeyboardHook|, leaving out some state
  // guards.
  void KeyboardHookImpl(int key,
                        int scancode,
                        int action,
                        char32_t character,
                        bool extended,
                        bool was_down,
                        std::function<void(bool)> callback);

  // Assign |critical_keys_| with basic information.
  void InitCriticalKeys(MapVirtualKeyToScanCode map_virtual_key_to_scan_code);
  // Update |critical_keys_| with last seen logical and physical key.
  void UpdateLastSeenCriticalKey(int virtual_key,
                                 uint64_t physical_key,
                                 uint64_t logical_key);
  // Check each key's state from |get_key_state_| and synthesize events
  // if their toggling states have been desynchronized.
  void SynchronizeCriticalToggledStates(int event_virtual_key,
                                        bool is_event_down,
                                        bool* event_key_can_be_repeat);
  // Check each key's state from |get_key_state_| and synthesize events
  // if their pressing states have been desynchronized.
  void SynchronizeCriticalPressedStates(int event_virtual_key,
                                        int event_physical_key,
                                        bool is_event_down,
                                        bool event_key_can_be_repeat);

  // Wraps perform_send_event_ with state tracking. Use this instead of
  // |perform_send_event_| to send events to the framework.
  void SendEvent(const FlutterKeyEvent& event,
                 FlutterKeyEventCallback callback,
                 void* user_data);

  // Send a synthesized down event and update pressing records.
  void SendSynthesizeDownEvent(uint64_t physical, uint64_t logical);

  // Send a synthesized up event and update pressing records.
  void SendSynthesizeUpEvent(uint64_t physical, uint64_t logical);

  // Send a synthesized up or down event depending on the current pressing
  // state.
  void SynthesizeIfNeeded(uint64_t physical_left,
                          uint64_t physical_right,
                          uint64_t logical_left,
                          bool is_pressed);

  std::function<void(const FlutterKeyEvent&, FlutterKeyEventCallback, void*)>
      perform_send_event_;
  GetKeyStateHandler get_key_state_;

  // A map from physical keys to logical keys, each entry indicating a pressed
  // key.
  std::map<uint64_t, uint64_t> pressingRecords_;
  // Information for key events that have been sent to the framework but yet
  // to receive the response. Indexed by response IDs.
  std::map<uint64_t, std::unique_ptr<PendingResponse>> pending_responses_;
  // A self-incrementing integer, used as the ID for the next entry for
  // |pending_responses_|.
  uint64_t response_id_;
  // Whether any events has been sent with |PerformSendEvent| during a
  // |KeyboardHook|.
  bool sent_any_events;

  // Important keys whose states are checked and guaranteed synchronized
  // on every key event.
  //
  // The following maps map Win32 virtual key to the physical key and logical
  // key they're last seen.
  std::map<UINT, CriticalKey> critical_keys_;

  static uint64_t GetPhysicalKey(int scancode, bool extended);
  static uint64_t GetLogicalKey(int key, bool extended, int scancode);
  static void HandleResponse(bool handled, void* user_data);
  static void ConvertUtf32ToUtf8_(char* out, char32_t ch);
  static FlutterKeyEvent SynthesizeSimpleEvent(FlutterKeyEventType type,
                                               uint64_t physical,
                                               uint64_t logical,
                                               const char* character);
  static uint64_t ApplyPlaneToId(uint64_t id, uint64_t plane);

  static std::map<uint64_t, uint64_t> windowsToPhysicalMap_;
  static std::map<uint64_t, uint64_t> windowsToLogicalMap_;
  static std::map<uint64_t, uint64_t> scanCodeToLogicalMap_;

  // Mask for the 32-bit value portion of the key code.
  static const uint64_t valueMask;

  // The plane value for keys which have a Unicode representation.
  static const uint64_t unicodePlane;

  // The plane value for the private keys defined by the GTK embedding.
  static const uint64_t windowsPlane;

  FML_DISALLOW_COPY_AND_ASSIGN(KeyboardKeyEmbedderHandler);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_KEYBOARD_KEY_EMBEDDER_HANDLER_H_
