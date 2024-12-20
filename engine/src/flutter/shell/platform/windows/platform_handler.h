// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_PLATFORM_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_PLATFORM_HANDLER_H_

#include <Windows.h>

#include <functional>
#include <memory>
#include <optional>
#include <variant>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/binary_messenger.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_channel.h"
#include "rapidjson/document.h"

namespace flutter {

class FlutterWindowsEngine;
class ScopedClipboardInterface;

// Indicates whether an exit request may be canceled by the framework.
// These values must be kept in sync with kExitTypeNames in platform_handler.cc
enum class AppExitType {
  required,
  cancelable,
};

// Handler for internal system channels.
class PlatformHandler {
 public:
  explicit PlatformHandler(
      BinaryMessenger* messenger,
      FlutterWindowsEngine* engine,
      std::optional<std::function<std::unique_ptr<ScopedClipboardInterface>()>>
          scoped_clipboard_provider = std::nullopt);

  virtual ~PlatformHandler();

  // String values used for encoding/decoding exit requests.
  static constexpr char kExitTypeCancelable[] = "cancelable";
  static constexpr char kExitTypeRequired[] = "required";

  // Send a request to the framework to test if a cancelable exit request
  // should be canceled or honored. hwnd is std::nullopt for a request to quit
  // the process, otherwise it holds the HWND of the window that initiated the
  // quit request.
  virtual void RequestAppExit(std::optional<HWND> hwnd,
                              std::optional<WPARAM> wparam,
                              std::optional<LPARAM> lparam,
                              AppExitType exit_type,
                              UINT exit_code);

 protected:
  // Gets plain text from the clipboard and provides it to |result| as the
  // value in a dictionary with the given |key|.
  virtual void GetPlainText(
      std::unique_ptr<MethodResult<rapidjson::Document>> result,
      std::string_view key);

  // Provides a boolean to |result| as the value in a dictionary at key
  // "value" representing whether or not the clipboard has a non-empty string.
  virtual void GetHasStrings(
      std::unique_ptr<MethodResult<rapidjson::Document>> result);

  // Sets the clipboard's plain text to |text|, and reports the result (either
  // an error, or null for success) to |result|.
  virtual void SetPlainText(
      const std::string& text,
      std::unique_ptr<MethodResult<rapidjson::Document>> result);

  virtual void SystemSoundPlay(
      const std::string& sound_type,
      std::unique_ptr<MethodResult<rapidjson::Document>> result);

  // Handle a request from the framework to exit the application.
  virtual void SystemExitApplication(
      AppExitType exit_type,
      UINT exit_code,
      std::unique_ptr<MethodResult<rapidjson::Document>> result);

  // Actually quit the application with the provided exit code. hwnd is
  // std::nullopt for a request to quit the process, otherwise it holds the HWND
  // of the window that initiated the quit request.
  virtual void QuitApplication(std::optional<HWND> hwnd,
                               std::optional<WPARAM> wparam,
                               std::optional<LPARAM> lparam,
                               UINT exit_code);

  // Callback from when the cancelable exit request response request is
  // answered by the framework. hwnd is std::nullopt for a request to quit the
  // process, otherwise it holds the HWND of the window that initiated the quit
  // request.
  virtual void RequestAppExitSuccess(std::optional<HWND> hwnd,
                                     std::optional<WPARAM> wparam,
                                     std::optional<LPARAM> lparam,
                                     const rapidjson::Document* result,
                                     UINT exit_code);

  // A error type to use for error responses.
  static constexpr char kClipboardError[] = "Clipboard error";

  static constexpr char kSoundTypeAlert[] = "SystemSoundType.alert";

 private:
  // Called when a method is called on |channel_|;
  void HandleMethodCall(
      const MethodCall<rapidjson::Document>& method_call,
      std::unique_ptr<MethodResult<rapidjson::Document>> result);

  // The MethodChannel used for communication with the Flutter engine.
  std::unique_ptr<MethodChannel<rapidjson::Document>> channel_;

  // A reference to the Flutter engine.
  FlutterWindowsEngine* engine_;

  // A scoped clipboard provider that can be passed in for mocking in tests.
  // Use this to acquire clipboard in each operation to avoid blocking clipboard
  // unnecessarily. See flutter/flutter#103205.
  std::function<std::unique_ptr<ScopedClipboardInterface>()>
      scoped_clipboard_provider_;

  FML_DISALLOW_COPY_AND_ASSIGN(PlatformHandler);
};

// A public interface for ScopedClipboard, so that it can be injected into
// PlatformHandler.
class ScopedClipboardInterface {
 public:
  virtual ~ScopedClipboardInterface(){};

  // Attempts to open the clipboard for the given window, returning the error
  // code in the case of failure and 0 otherwise.
  virtual int Open(HWND window) = 0;

  // Returns true if there is string data available to get.
  virtual bool HasString() = 0;

  // Returns string data from the clipboard.
  //
  // If getting a string fails, returns the error code.
  //
  // Open(...) must have succeeded to call this method.
  virtual std::variant<std::wstring, int> GetString() = 0;

  // Sets the string content of the clipboard, returning the error code on
  // failure and 0 otherwise.
  //
  // Open(...) must have succeeded to call this method.
  virtual int SetString(const std::wstring string) = 0;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_PLATFORM_HANDLER_H_
