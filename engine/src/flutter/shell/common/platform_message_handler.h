// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_PLATFORM_MESSAGE_HANDLER_H_
#define FLUTTER_SHELL_COMMON_PLATFORM_MESSAGE_HANDLER_H_

#include <memory>

#include "flutter/fml/mapping.h"

namespace flutter {

class PlatformMessage;

/// An interface over the ability to handle PlatformMessages that are being sent
/// from Flutter to the host platform.
class PlatformMessageHandler {
 public:
  virtual ~PlatformMessageHandler() = default;

  /// Ultimately sends the PlatformMessage to the host platform.
  /// This method is invoked on the ui thread.
  virtual void HandlePlatformMessage(
      std::unique_ptr<PlatformMessage> message) = 0;

  /// Returns true if the platform message will ALWAYS be dispatched to the
  /// platform thread.
  ///
  /// Platforms that support Background Platform Channels will return
  /// false.
  virtual bool DoesHandlePlatformMessageOnPlatformThread() const = 0;

  /// Performs the return procedure for an associated call to
  /// HandlePlatformMessage.
  /// This method should be thread-safe and able to be invoked on any thread.
  virtual void InvokePlatformMessageResponseCallback(
      int response_id,
      std::unique_ptr<fml::Mapping> mapping) = 0;

  /// Performs the return procedure for an associated call to
  /// HandlePlatformMessage where there is no return value.
  /// This method should be thread-safe and able to be invoked on any thread.
  virtual void InvokePlatformMessageEmptyResponseCallback(int response_id) = 0;
};
}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_PLATFORM_MESSAGE_HANDLER_H_
