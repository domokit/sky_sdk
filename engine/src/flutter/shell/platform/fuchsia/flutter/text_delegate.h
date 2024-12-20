// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Text editing functionality delegated from |PlatformView|.
/// See |TextDelegate| for details.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TEXT_DELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TEXT_DELEGATE_H_

#include <memory>

#include <fuchsia/ui/input/cpp/fidl.h>
#include <fuchsia/ui/input3/cpp/fidl.h>
#include <fuchsia/ui/views/cpp/fidl.h>
#include <lib/fidl/cpp/binding.h>

#include "flutter/lib/ui/window/platform_message.h"
#include "flutter/shell/common/platform_view.h"
#include "flutter/shell/platform/fuchsia/flutter/keyboard.h"

#include "logging.h"

namespace flutter_runner {

/// The channel name used for text editing platofrm messages.
constexpr char kTextInputChannel[] = "flutter/textinput";

/// The channel name used for key event platform messages.
constexpr char kKeyEventChannel[] = "flutter/keyevent";

/// TextDelegate handles keyboard inpout and text editing.
///
/// It mediates between Fuchsia's input and Flutter's platform messages. When it
/// is initialized, it contacts `fuchsia.ui.input.Keyboard` to register itself
/// as listener of key events.
///
/// Whenever a text editing request comes from the
/// Flutter app, it will activate Fuchsia's input method editor, and will send
/// text edit actions coming from the Fuchsia platform over to the Flutter app,
/// by converting FIDL messages (`fuchsia.ui.input.InputMethodEditorClient`
/// calls) to appropriate text editing Flutter platform messages.
///
/// For details refer to:
///   * Flutter side:
///   https://api.flutter.dev/javadoc/io/flutter/embedding/engine/systemchannels/TextInputChannel.html
///   * Fuchsia side: https://fuchsia.dev/reference/fidl/fuchsia.ui.input
class TextDelegate : public fuchsia::ui::input3::KeyboardListener,
                     public fuchsia::ui::input::InputMethodEditorClient {
 public:
  /// Creates a new TextDelegate.
  ///
  /// Args:
  ///   view_ref: the reference to the app's view. Required for registration
  ///     with Fuchsia.
  ///   ime_service: a handle to Fuchsia's input method service.
  ///   keyboard: the keyboard listener, gets notified of key presses and
  ///     releases.
  ///   dispatch_callback: a function used to send a Flutter platform message.
  TextDelegate(fuchsia::ui::views::ViewRef view_ref,
               fuchsia::ui::input::ImeServiceHandle ime_service,
               fuchsia::ui::input3::KeyboardHandle keyboard,
               std::function<void(std::unique_ptr<flutter::PlatformMessage>)>
                   dispatch_callback);

  /// |fuchsia.ui.input3.KeyboardListener|
  /// Called by the embedder every time there is a key event to process.
  void OnKeyEvent(fuchsia::ui::input3::KeyEvent key_event,
                  fuchsia::ui::input3::KeyboardListener::OnKeyEventCallback
                      callback) override;

  /// |fuchsia::ui::input::InputMethodEditorClient|
  /// Called by the embedder every time the edit state is updated.
  void DidUpdateState(
      fuchsia::ui::input::TextInputState state,
      std::unique_ptr<fuchsia::ui::input::InputEvent> event) override;

  /// |fuchsia::ui::input::InputMethodEditorClient|
  /// Called by the embedder when the action key is pressed, and the requested
  /// action is supplied to Flutter.
  void OnAction(fuchsia::ui::input::InputMethodAction action) override;

  /// Gets a new input method editor from the input connection. Run when both
  /// Scenic has focus and Flutter has requested input with setClient.
  void ActivateIme();

  /// Detaches the input method editor connection, ending the edit session and
  /// closing the onscreen keyboard. Call when input is no longer desired,
  /// either because Scenic says we lost focus or when Flutter no longer has a
  /// text field focused.
  void DeactivateIme();

  /// Channel handler for kTextInputChannel
  bool HandleFlutterTextInputChannelPlatformMessage(
      std::unique_ptr<flutter::PlatformMessage> message);

  /// Returns true if there is a text state (i.e. if some text editing is in
  /// progress).
  bool HasTextState() { return last_text_state_.has_value(); }

 private:
  // Activates the input method editor, assigning |action| to the "enter" key.
  // This action will be reported by |OnAction| above when the "enter" key is
  // pressed. Note that in the case of multi-line text editors, |OnAction| will
  // never be called: instead, the text editor will insert a newline into the
  // edited text.
  void ActivateIme(fuchsia::ui::input::InputMethodAction action);

  // Converts Fuchsia platform key codes into Flutter key codes.
  Keyboard keyboard_translator_;

  // A callback for sending a single Flutter platform message.
  std::function<void(std::unique_ptr<flutter::PlatformMessage>)>
      dispatch_callback_;

  // TextDelegate server-side binding.  Methods called when the text edit state
  // is updated.
  fidl::Binding<fuchsia::ui::input::InputMethodEditorClient> ime_client_;

  // An interface for interacting with a text input control.
  fuchsia::ui::input::InputMethodEditorPtr ime_;

  // An interface for requesting the InputMethodEditor.
  fuchsia::ui::input::ImeServicePtr text_sync_service_;

  // The locally-unique identifier of the text input currently in use. Flutter
  // usually uses only one at a time.
  int current_text_input_client_ = 0;

  // TextDelegate server side binding. Methods called when a key is pressed.
  fidl::Binding<fuchsia::ui::input3::KeyboardListener>
      keyboard_listener_binding_;

  // The client-side stub for calling the Keyboard protocol.
  fuchsia::ui::input3::KeyboardPtr keyboard_;

  // last_text_state_ is the last state of the text input as reported by the IME
  // or initialized by Flutter. We set it to null if Flutter doesn't want any
  // input, since then there is no text input state at all.
  // If nullptr, then no editing is in progress.
  std::optional<fuchsia::ui::input::TextInputState> last_text_state_;

  // The action that Flutter expects to happen when the user presses the "enter"
  // key.  For example, it could be `InputMethodAction::DONE` which would cause
  // text editing to stop and the current text to be accepted.
  // If set to std::nullopt, then no editing is in progress.
  std::optional<fuchsia::ui::input::InputMethodAction> requested_text_action_;

  FML_DISALLOW_COPY_AND_ASSIGN(TextDelegate);
};

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TEXT_DELEGATE_H_
