// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_WINDOWS_ENGINE_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_WINDOWS_ENGINE_H_

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "flutter/fml/closure.h"
#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/accessibility_bridge.h"
#include "flutter/shell/platform/common/app_lifecycle_state.h"
#include "flutter/shell/platform/common/client_wrapper/binary_messenger_impl.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/basic_message_channel.h"
#include "flutter/shell/platform/common/incoming_message_dispatcher.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/windows/accessibility_bridge_windows.h"
#include "flutter/shell/platform/windows/accessibility_plugin.h"
#include "flutter/shell/platform/windows/compositor.h"
#include "flutter/shell/platform/windows/cursor_handler.h"
#include "flutter/shell/platform/windows/egl/manager.h"
#include "flutter/shell/platform/windows/egl/proc_table.h"
#include "flutter/shell/platform/windows/flutter_desktop_messenger.h"
#include "flutter/shell/platform/windows/flutter_project_bundle.h"
#include "flutter/shell/platform/windows/flutter_windows_texture_registrar.h"
#include "flutter/shell/platform/windows/keyboard_handler_base.h"
#include "flutter/shell/platform/windows/keyboard_key_embedder_handler.h"
#include "flutter/shell/platform/windows/platform_handler.h"
#include "flutter/shell/platform/windows/platform_view_plugin.h"
#include "flutter/shell/platform/windows/public/flutter_windows.h"
#include "flutter/shell/platform/windows/settings_plugin.h"
#include "flutter/shell/platform/windows/task_runner.h"
#include "flutter/shell/platform/windows/text_input_plugin.h"
#include "flutter/shell/platform/windows/window_proc_delegate_manager.h"
#include "flutter/shell/platform/windows/window_state.h"
#include "flutter/shell/platform/windows/windows_lifecycle_manager.h"
#include "flutter/shell/platform/windows/windows_proc_table.h"
#include "third_party/rapidjson/include/rapidjson/document.h"

namespace flutter {

// The implicit view's ID.
//
// See:
// https://api.flutter.dev/flutter/dart-ui/PlatformDispatcher/implicitView.html
constexpr FlutterViewId kImplicitViewId = 0;

class FlutterWindowsView;

// Update the thread priority for the Windows engine.
static void WindowsPlatformThreadPrioritySetter(
    FlutterThreadPriority priority) {
  // TODO(99502): Add support for tracing to the windows embedding so we can
  // mark thread priorities and success/failure.
  switch (priority) {
    case FlutterThreadPriority::kBackground: {
      SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
      break;
    }
    case FlutterThreadPriority::kDisplay: {
      SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
      break;
    }
    case FlutterThreadPriority::kRaster: {
      SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
      break;
    }
    case FlutterThreadPriority::kNormal: {
      // For normal or default priority we do not need to set the priority
      // class.
      break;
    }
  }
}

// Manages state associated with the underlying FlutterEngine that isn't
// related to its display.
//
// In most cases this will be associated with a FlutterView, but if not will
// run in headless mode.
class FlutterWindowsEngine {
 public:
  // Creates a new Flutter engine object configured to run |project|.
  FlutterWindowsEngine(
      const FlutterProjectBundle& project,
      std::shared_ptr<WindowsProcTable> windows_proc_table = nullptr);

  virtual ~FlutterWindowsEngine();

  // Starts running the entrypoint function specifed in the project bundle. If
  // unspecified, defaults to main().
  //
  // Returns false if the engine couldn't be started.
  bool Run();

  // Starts running the engine with the given entrypoint. If the empty string
  // is specified, defaults to the entrypoint function specified in the project
  // bundle, or main() if both are unspecified.
  //
  // Returns false if the engine couldn't be started or if conflicting,
  // non-default values are passed here and in the project bundle..
  //
  // DEPRECATED: Prefer setting the entrypoint in the FlutterProjectBundle
  // passed to the constructor and calling the no-parameter overload.
  bool Run(std::string_view entrypoint);

  // Returns true if the engine is currently running.
  virtual bool running() const { return engine_ != nullptr; }

  // Stops the engine. This invalidates the pointer returned by engine().
  //
  // Returns false if stopping the engine fails, or if it was not running.
  virtual bool Stop();

  // Create a view that can display this engine's content.
  //
  // Returns null on failure.
  std::unique_ptr<FlutterWindowsView> CreateView(
      std::unique_ptr<WindowBindingHandler> window);

  // Remove a view. The engine will no longer render into it.
  virtual void RemoveView(FlutterViewId view_id);

  // Get a view that displays this engine's content.
  //
  // Returns null if the view does not exist.
  FlutterWindowsView* view(FlutterViewId view_id) const;

  // Returns the currently configured Plugin Registrar.
  FlutterDesktopPluginRegistrarRef GetRegistrar();

  // Registers |callback| to be called when the plugin registrar is destroyed.
  void AddPluginRegistrarDestructionCallback(
      FlutterDesktopOnPluginRegistrarDestroyed callback,
      FlutterDesktopPluginRegistrarRef registrar);

  // Sets switches member to the given switches.
  void SetSwitches(const std::vector<std::string>& switches);

  FlutterDesktopMessengerRef messenger() { return messenger_->ToRef(); }

  IncomingMessageDispatcher* message_dispatcher() {
    return message_dispatcher_.get();
  }

  TaskRunner* task_runner() { return task_runner_.get(); }

  BinaryMessenger* messenger_wrapper() { return messenger_wrapper_.get(); }

  FlutterWindowsTextureRegistrar* texture_registrar() {
    return texture_registrar_.get();
  }

  // The EGL manager object. If this is nullptr, then we are
  // rendering using software instead of OpenGL.
  egl::Manager* egl_manager() const { return egl_manager_.get(); }

  WindowProcDelegateManager* window_proc_delegate_manager() {
    return window_proc_delegate_manager_.get();
  }

  // Informs the engine that the window metrics have changed.
  void SendWindowMetricsEvent(const FlutterWindowMetricsEvent& event);

  // Informs the engine of an incoming pointer event.
  void SendPointerEvent(const FlutterPointerEvent& event);

  // Informs the engine of an incoming key event.
  void SendKeyEvent(const FlutterKeyEvent& event,
                    FlutterKeyEventCallback callback,
                    void* user_data);

  KeyboardHandlerBase* keyboard_key_handler() {
    return keyboard_key_handler_.get();
  }
  TextInputPlugin* text_input_plugin() { return text_input_plugin_.get(); }

  // Sends the given message to the engine, calling |reply| with |user_data|
  // when a response is received from the engine if they are non-null.
  bool SendPlatformMessage(const char* channel,
                           const uint8_t* message,
                           const size_t message_size,
                           const FlutterDesktopBinaryReply reply,
                           void* user_data);

  // Sends the given data as the response to an earlier platform message.
  void SendPlatformMessageResponse(
      const FlutterDesktopMessageResponseHandle* handle,
      const uint8_t* data,
      size_t data_length);

  // Callback passed to Flutter engine for notifying window of platform
  // messages.
  void HandlePlatformMessage(const FlutterPlatformMessage*);

  // Informs the engine that the system font list has changed.
  void ReloadSystemFonts();

  // Informs the engine that a new frame is needed to redraw the content.
  void ScheduleFrame();

  // Set the callback that is called when the next frame is drawn.
  void SetNextFrameCallback(fml::closure callback);

  // Attempts to register the texture with the given |texture_id|.
  bool RegisterExternalTexture(int64_t texture_id);

  // Attempts to unregister the texture with the given |texture_id|.
  bool UnregisterExternalTexture(int64_t texture_id);

  // Notifies the engine about a new frame being available for the
  // given |texture_id|.
  bool MarkExternalTextureFrameAvailable(int64_t texture_id);

  // Posts the given callback onto the raster thread.
  virtual bool PostRasterThreadTask(fml::closure callback) const;

  // Invoke on the embedder's vsync callback to schedule a frame.
  void OnVsync(intptr_t baton);

  // Dispatches a semantics action to the specified semantics node.
  bool DispatchSemanticsAction(uint64_t id,
                               FlutterSemanticsAction action,
                               fml::MallocMapping data);

  // Informs the engine that the semantics enabled state has changed.
  void UpdateSemanticsEnabled(bool enabled);

  // Returns true if the semantics tree is enabled.
  bool semantics_enabled() const { return semantics_enabled_; }

  // Refresh accessibility features and send them to the engine.
  void UpdateAccessibilityFeatures();

  // Refresh high contrast accessibility mode and notify the engine.
  void UpdateHighContrastMode();

  // Returns true if the high contrast feature is enabled.
  bool high_contrast_enabled() const { return high_contrast_enabled_; }

  // Register a root isolate create callback.
  //
  // The root isolate create callback is invoked at creation of the root Dart
  // isolate in the app. This may be used to be notified that execution of the
  // main Dart entrypoint is about to begin, and is used by test infrastructure
  // to register a native function resolver that can register and resolve
  // functions marked as native in the Dart code.
  //
  // This must be called before calling |Run|.
  void SetRootIsolateCreateCallback(const fml::closure& callback) {
    root_isolate_create_callback_ = callback;
  }

  // Returns the executable name for this process or "Flutter" if unknown.
  std::string GetExecutableName() const;

  // Called when the application quits in response to a quit request.
  void OnQuit(std::optional<HWND> hwnd,
              std::optional<WPARAM> wparam,
              std::optional<LPARAM> lparam,
              UINT exit_code);

  // Called when a WM_CLOSE message is received.
  void RequestApplicationQuit(HWND hwnd,
                              WPARAM wparam,
                              LPARAM lparam,
                              AppExitType exit_type);

  // Called when a WM_DWMCOMPOSITIONCHANGED message is received.
  void OnDwmCompositionChanged();

  // Called when a Window receives an event that may alter the application
  // lifecycle state.
  void OnWindowStateEvent(HWND hwnd, WindowStateEvent event);

  // Handle a message from a non-Flutter window in the same application.
  // Returns a result when the message is consumed and should not be processed
  // further.
  std::optional<LRESULT> ProcessExternalWindowMessage(HWND hwnd,
                                                      UINT message,
                                                      WPARAM wparam,
                                                      LPARAM lparam);

  WindowsLifecycleManager* lifecycle_manager() {
    return lifecycle_manager_.get();
  }

  std::shared_ptr<WindowsProcTable> windows_proc_table() {
    return windows_proc_table_;
  }

 protected:
  // Creates the keyboard key handler.
  //
  // Exposing this method allows unit tests to override in order to
  // capture information.
  virtual std::unique_ptr<KeyboardHandlerBase> CreateKeyboardKeyHandler(
      BinaryMessenger* messenger,
      KeyboardKeyEmbedderHandler::GetKeyStateHandler get_key_state,
      KeyboardKeyEmbedderHandler::MapVirtualKeyToScanCode map_vk_to_scan);

  // Creates the text input plugin.
  //
  // Exposing this method allows unit tests to override in order to
  // capture information.
  virtual std::unique_ptr<TextInputPlugin> CreateTextInputPlugin(
      BinaryMessenger* messenger);

  // Invoked by the engine right before the engine is restarted.
  //
  // This should reset necessary states to as if the engine has just been
  // created. This is typically caused by a hot restart (Shift-R in CLI.)
  void OnPreEngineRestart();

  // Invoked by the engine when a listener is set or cleared on a platform
  // channel.
  virtual void OnChannelUpdate(std::string name, bool listening);

 private:
  // Allows swapping out embedder_api_ calls in tests.
  friend class EngineModifier;

  // Sends system locales to the engine.
  //
  // Should be called just after the engine is run, and after any relevant
  // system changes.
  void SendSystemLocales();

  // Sends the current lifecycle state to the framework.
  void SetLifecycleState(flutter::AppLifecycleState state);

  // Create the keyboard & text input sub-systems.
  //
  // This requires that a view is attached to the engine.
  // Calling this method again resets the keyboard state.
  void InitializeKeyboard();

  // Send the currently enabled accessibility features to the engine.
  void SendAccessibilityFeatures();

  // Present content to a view. Returns true if the content was presented.
  //
  // This is invoked on the raster thread.
  bool Present(const FlutterPresentViewInfo* info);

  // The handle to the embedder.h engine instance.
  FLUTTER_API_SYMBOL(FlutterEngine) engine_ = nullptr;

  FlutterEngineProcTable embedder_api_ = {};

  std::unique_ptr<FlutterProjectBundle> project_;

  // AOT data, if any.
  UniqueAotDataPtr aot_data_;

  // The ID that the next view will have.
  FlutterViewId next_view_id_ = kImplicitViewId;

  // The views displaying the content running in this engine, if any.
  //
  // This is read and mutated by the platform thread. This is read by the raster
  // thread to present content to a view.
  //
  // Reads to this object on non-platform threads must be protected
  // by acquiring a shared lock on |views_mutex_|.
  //
  // Writes to this object must only happen on the platform thread
  // and must be protected by acquiring an exclusive lock on |views_mutex_|.
  std::unordered_map<FlutterViewId, FlutterWindowsView*> views_;

  // The mutex that protects the |views_| map.
  //
  // The raster thread acquires a shared lock to present to a view.
  //
  // The platform thread acquires a shared lock to access the view.
  // The platform thread acquires an exclusive lock before adding
  // a view to the engine or after removing a view from the engine.
  mutable std::shared_mutex views_mutex_;

  // Task runner for tasks posted from the engine.
  std::unique_ptr<TaskRunner> task_runner_;

  // The plugin messenger handle given to API clients.
  fml::RefPtr<flutter::FlutterDesktopMessenger> messenger_;

  // A wrapper around messenger_ for interacting with client_wrapper-level APIs.
  std::unique_ptr<BinaryMessengerImpl> messenger_wrapper_;

  // Message dispatch manager for messages from engine_.
  std::unique_ptr<IncomingMessageDispatcher> message_dispatcher_;

  // The plugin registrar handle given to API clients.
  std::unique_ptr<FlutterDesktopPluginRegistrar> plugin_registrar_;

  // The texture registrar.
  std::unique_ptr<FlutterWindowsTextureRegistrar> texture_registrar_;

  // An object used for intializing ANGLE and creating / destroying render
  // surfaces. If nullptr, ANGLE failed to initialize and software rendering
  // should be used instead.
  std::unique_ptr<egl::Manager> egl_manager_;

  // The compositor that creates backing stores for the engine to render into
  // and then presents them onto views.
  std::unique_ptr<Compositor> compositor_;

  // The plugin registrar managing internal plugins.
  std::unique_ptr<PluginRegistrar> internal_plugin_registrar_;

  // Handler for accessibility events.
  std::unique_ptr<AccessibilityPlugin> accessibility_plugin_;

  // Handler for cursor events.
  std::unique_ptr<CursorHandler> cursor_handler_;

  // Handler for the flutter/platform channel.
  std::unique_ptr<PlatformHandler> platform_handler_;

  // Handlers for keyboard events from Windows.
  std::unique_ptr<KeyboardHandlerBase> keyboard_key_handler_;

  // Handlers for text events from Windows.
  std::unique_ptr<TextInputPlugin> text_input_plugin_;

  // The settings plugin.
  std::unique_ptr<SettingsPlugin> settings_plugin_;

  // Callbacks to be called when the engine (and thus the plugin registrar) is
  // being destroyed.
  std::map<FlutterDesktopOnPluginRegistrarDestroyed,
           FlutterDesktopPluginRegistrarRef>
      plugin_registrar_destruction_callbacks_;

  // The approximate time between vblank events.
  std::chrono::nanoseconds FrameInterval();

  // The start time used to align frames.
  std::chrono::nanoseconds start_time_ = std::chrono::nanoseconds::zero();

  // An override of the frame interval used by EngineModifier for testing.
  std::optional<std::chrono::nanoseconds> frame_interval_override_ =
      std::nullopt;

  bool semantics_enabled_ = false;

  bool high_contrast_enabled_ = false;

  bool enable_impeller_ = false;

  // The manager for WindowProc delegate registration and callbacks.
  std::unique_ptr<WindowProcDelegateManager> window_proc_delegate_manager_;

  // The root isolate creation callback.
  fml::closure root_isolate_create_callback_;

  // The on frame drawn callback.
  fml::closure next_frame_callback_;

  // Handler for top level window messages.
  std::unique_ptr<WindowsLifecycleManager> lifecycle_manager_;

  std::shared_ptr<WindowsProcTable> windows_proc_table_;

  std::shared_ptr<egl::ProcTable> gl_;

  std::unique_ptr<PlatformViewPlugin> platform_view_plugin_;

  FML_DISALLOW_COPY_AND_ASSIGN(FlutterWindowsEngine);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_WINDOWS_ENGINE_H_
