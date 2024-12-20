// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_BINARY_MESSENGER_IMPL_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_BINARY_MESSENGER_IMPL_H_

#include <flutter_messenger.h>

#include <map>
#include <string>

#include "include/flutter/binary_messenger.h"

namespace flutter {

// Wrapper around a FlutterDesktopMessengerRef that implements the
// BinaryMessenger API.
class BinaryMessengerImpl : public BinaryMessenger {
 public:
  explicit BinaryMessengerImpl(FlutterDesktopMessengerRef core_messenger);

  virtual ~BinaryMessengerImpl();

  // Prevent copying.
  BinaryMessengerImpl(BinaryMessengerImpl const&) = delete;
  BinaryMessengerImpl& operator=(BinaryMessengerImpl const&) = delete;

  // |flutter::BinaryMessenger|
  void Send(const std::string& channel,
            const uint8_t* message,
            size_t message_size,
            BinaryReply reply) const override;

  // |flutter::BinaryMessenger|
  void SetMessageHandler(const std::string& channel,
                         BinaryMessageHandler handler) override;

 private:
  // Handle for interacting with the C API.
  FlutterDesktopMessengerRef messenger_;

  // A map from channel names to the BinaryMessageHandler that should be called
  // for incoming messages on that channel.
  std::map<std::string, BinaryMessageHandler> handlers_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_BINARY_MESSENGER_IMPL_H_
